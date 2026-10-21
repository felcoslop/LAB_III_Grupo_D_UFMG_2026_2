// teste_calibracao.mjs
// Confere a conta da calibração de pista (bloco CAL_PISTA_MATEMATICA do painel/painel_src.html) com dados
// sintéticos: pista com arranhão, módulo invertido, sinal digital (fio no D0), dados de menos, par fora da faixa,
// preto sujo demais. Não precisa de navegador: o bloco só tem números.
// Uso: node teste/painel/teste_calibracao.mjs
import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const aqui = path.dirname(fileURLToPath(import.meta.url));
const html = fs.readFileSync(path.resolve(aqui, "..", "..", "painel", "painel_src.html"), "utf-8");
const a = html.indexOf("// <CAL_PISTA_MATEMATICA>"), b = html.indexOf("// </CAL_PISTA_MATEMATICA>");
if (a < 0 || b < a) { console.log("FALHA  bloco CAL_PISTA_MATEMATICA não encontrado no painel_src.html"); process.exit(1); }
const { CP, cpSensor, cpAnalisa, cpQuantil } = new Function(html.slice(a, b) + "\nreturn {CP, cpSensor, cpAnalisa, cpQuantil};")();

let falhas = 0;
const confere = (ok, oque, extra = "") => { console.log(`${ok ? "ok   " : "FALHA"}  ${oque}${extra ? "  " + extra : ""}`); if (!ok) falhas++; };

let rng = 987654321;
const rand = () => { rng = (rng * 1664525 + 1013904223) >>> 0; return rng / 4294967296; };
const gauss = () => { let s = 0; for (let k = 0; k < 6; k++) s += rand(); return (s - 3) / 0.7071; };

// uma janela de 20 ms de um sensor: 20 leituras com ruído
function janela(nivel, ruido, { invertido = false, digital = false, arranhao = 0, arranhaoMv = 0, sujeira = 0, sujeiraMv = 0, branco = false } = {}) {
  let mn = Infinity, mx = -Infinity, s = 0;
  for (let k = 0; k < 20; k++) {
    let x;
    if (digital) x = branco ? 0 : 3300;
    else {
      x = nivel + gauss() * ruido;
      if (!branco && rand() < arranhao / 20) x = arranhaoMv + gauss() * ruido;
      if (branco && rand() < sujeira / 20) x = sujeiraMv + gauss() * ruido;
      if (invertido) x = 4095 - x;
      x = Math.max(0, Math.min(3300, Math.round(x)));
    }
    mn = Math.min(mn, x); mx = Math.max(mx, x); s += x;
  }
  return { mn, mx, md: Math.round(s / 20) };
}
const junta = (js) => ({ mn: js.map((j) => j.mn), mx: js.map((j) => j.mx), md: js.map((j) => j.md) });
// monta {preto, passes} para 4 sensores: preto = n janelas; cada passe: par e n janelas (par na faixa, outros no preto)
function cena({ pretoMv = 2800, brancoMv = 300, ruido = 40, n = 400, pares = [[2, 3], [0, 1]], emFaixa = 1, ...opc } = {}) {
  const preto = [];
  for (let k = 0; k < n; k++) { const w = { mn: [], mx: [], md: [] }; for (let i = 0; i < 4; i++) { const j = janela(pretoMv, ruido, opc); w.mn.push(j.mn); w.mx.push(j.mx); w.md.push(j.md); } preto.push(w); }
  const passes = pares.map((par) => {
    const janelas = [];
    for (let k = 0; k < n; k++) {
      const w = { mn: [], mx: [], md: [] };
      const sobre = rand() < emFaixa;     // com emFaixa < 1 o par escorrega para fora da faixa parte do tempo
      for (let i = 0; i < 4; i++) { const branco = par.includes(i) && sobre; const j = janela(branco ? brancoMv : pretoMv, ruido, { ...opc, branco }); w.mn.push(j.mn); w.mx.push(j.mx); w.md.push(j.md); }
      janelas.push(w);
    }
    return { par, janelas };
  });
  return { preto, passes };
}

// 1. pista normal (A0, branco = tensão baixa), arranhões claros no preto
{
  const { preto, passes } = cena({ arranhao: 0.4, arranhaoMv: 1700 });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => x.ok), "pista normal: os quatro sensores calculam", r.map((x) => x.motivo).join("|"));
  confere(r.every((x) => x.abaixo === true), "pista normal: branco = tensão baixa");
  confere(r.every((x) => x.limiar > x.piorBranco && x.limiar < x.piorPreto), "limiar entre o pior branco e o pior preto", r.map((x) => `${x.piorBranco}<${x.limiar}<${x.piorPreto}`).join(" "));
  confere(r.every((x) => Math.abs(x.margemBranco - x.margemPreto) <= 2), "fração 0,5: as duas margens ficam iguais", r.map((x) => `${x.margemBranco}/${x.margemPreto}`).join(" "));
  confere(r.every((x) => x.limiar > 600 && x.limiar < 2200), "limiar em uma faixa plausível (600 a 2200 mV)", r.map((x) => x.limiar).join(" "));
  const x0 = r[0];
  confere(x0.nFaixa >= 300, "janelas sobre a faixa contadas", String(x0.nFaixa));
}

// 2. mesmo com arranhão muito claro (abaixo do limiar) a conta avisa em vez de esconder
{
  const { preto, passes } = cena({ arranhao: 3, arranhaoMv: 400 });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => x.ok && x.fpJanelas > 0), "arranhão tão claro quanto a faixa: falso positivo aparece na conta", r.map((x) => `${x.fpEventos}/${x.fpJanelas}`).join(" "));
  confere(r.every((x) => x.nivel !== "boa"), "esse caso nunca sai como boa", r.map((x) => x.nivel).join(" "));
}

// 3. limiar acompanha a fração
{
  const { preto, passes } = cena();
  const claro = cpAnalisa(preto, passes, 4, 0.25), meio = cpAnalisa(preto, passes, 4, 0.5), escuro = cpAnalisa(preto, passes, 4, 0.75);
  confere(claro[0].limiar < meio[0].limiar && meio[0].limiar < escuro[0].limiar, "fração maior = limiar mais perto do preto", `${claro[0].limiar} < ${meio[0].limiar} < ${escuro[0].limiar}`);
}

// 4. módulo invertido: branco = tensão ALTA
{
  const { preto, passes } = cena({ invertido: true });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => x.ok && x.abaixo === false), "módulo invertido detectado", r.map((x) => x.abaixo).join(","));
  confere(r.every((x) => x.limiar > x.piorPreto && x.limiar < x.piorBranco), "invertido: limiar entre o pior preto (baixo) e o pior branco (alto)", r.map((x) => `${x.piorPreto}<${x.limiar}<${x.piorBranco}`).join(" "));
  confere(r.every((x) => x.margemBranco > 100 && x.margemPreto > 100), "invertido: margens positivas", r.map((x) => `${x.margemBranco}/${x.margemPreto}`).join(" "));
}

// 5. sinal digital (fio no D0): vira aviso, nível no máximo aceitável
{
  const { preto, passes } = cena({ digital: true });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => x.ok && x.digital), "D0: sinal digital reconhecido");
  confere(r.every((x) => x.nivel !== "boa" && x.avisos.some((t) => /D0/.test(t))), "D0: aviso explica o trimpot e o A0", r[0].avisos.join(" | ").slice(0, 120));
}

// 6. dados de menos
{
  const { preto, passes } = cena({ n: 60 });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => !x.ok && /poucos dados/.test(x.motivo)), "menos de 100 janelas: recusa e diz o motivo", r[0].motivo);
}

// 7. só o par traseiro foi passado: os frontais ficam sem dados e os traseiros calculam
{
  const { preto, passes } = cena({ pares: [[2, 3]] });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r[2].ok && r[3].ok && !r[0].ok && !r[1].ok, "só o par traseiro: traseiros ok, frontais sem passe");
  confere(/nenhum passe/.test(r[0].motivo), "o motivo dos frontais fala do passe", r[0].motivo);
}

// 8. par que quase não ficou na faixa
{
  const { preto, passes } = cena({ emFaixa: 0.15, n: 400 });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => !x.ok && /sobre a faixa/.test(x.motivo)), "par fora da faixa quase todo o tempo: recusa", r[0].motivo);
}

// 9. par metade do tempo na faixa: calcula, com aviso
{
  const { preto, passes } = cena({ emFaixa: 0.35, n: 800 });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => x.ok), "par 35% do tempo na faixa e 800 janelas: calcula", r.map((x) => x.motivo).join("|"));
  confere(r.every((x) => x.avisos.some((t) => /% do tempo sobre a faixa/.test(t))), "avisa que o par ficou pouco tempo na faixa");
  confere(r.every((x) => x.limiar > 800 && x.limiar < 2000), "e o limiar continua no meio", r.map((x) => x.limiar).join(" "));
}

// 10. preto sujo: preto 1100 mV com ruído enorme contra branco 800: sem folga
{
  const { preto, passes } = cena({ pretoMv: 1100, brancoMv: 800, ruido: 250 });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => x.ok && !x.separavel && x.nivel === "ruim"), "preto e branco sobrepostos: nível ruim, sem folga", r.map((x) => `${x.folga.toFixed(0)}:${x.nivel}`).join(" "));
  confere(r.every((x) => x.avisos.some((t) => /perto do pior branco/.test(t))), "aviso de arranhão ou sujeira");
}

// 11. contraste fraco (250 mV) mas separável
{
  const { preto, passes } = cena({ pretoMv: 700, brancoMv: 450, ruido: 15 });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => x.ok && x.separavel && x.nivel === "aceitável" && x.avisos.some((t) => /contraste fraco/.test(t))), "contraste de 250 mV: aceitável com aviso", r.map((x) => `${x.contraste.toFixed(0)}:${x.nivel}`).join(" "));
}

// 11b. arranhão claro estreita a folga: o aviso diz por quê (a nota "aceitável" nunca fica sem explicação)
{
  const { preto, passes } = cena({ arranhao: 1.2, arranhaoMv: 1200 });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => x.ok && x.nivel === "aceitável" && x.avisos.some((t) => /pequena frente ao contraste/.test(t))), "folga estreita: aceitável com explicação", r.map((x) => `${Math.round(x.folga)}:${x.nivel}`).join(" "));
}

// 12. limites do comando bordalim
{
  const { preto, passes } = cena({ pretoMv: 3250, brancoMv: 3000, ruido: 10, invertido: false });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  confere(r.every((x) => x.limiar >= CP.LIM_MIN_MV && x.limiar <= CP.LIM_MAX_MV), "limiar sempre dentro de 50 a 3300 mV", r.map((x) => x.limiar).join(" "));
}

// 13. quantil
confere(cpQuantil([1, 2, 3, 4, 5], 0.5) === 3 && cpQuantil([10], 0.9) === 10 && Number.isNaN(cpQuantil([], 0.5)), "quantil: mediana, um valor, vazio");

// 14. dado: preto mais branco muito separados nunca dá limiar fora do intervalo entre eles
for (let t = 0; t < 30; t++) {
  const pretoMv = 1500 + Math.floor(rand() * 1700), brancoMv = 50 + Math.floor(rand() * 700), ruido = 10 + Math.floor(rand() * 60);
  const { preto, passes } = cena({ pretoMv, brancoMv, ruido, n: 200 });
  const r = cpAnalisa(preto, passes, 4, 0.5);
  const fora = r.filter((x) => x.ok && (x.limiar <= brancoMv - 4 * ruido || x.limiar >= pretoMv + 4 * ruido));
  if (fora.length) { confere(false, `aleatório ${t}: limiar fora do intervalo`, `preto ${pretoMv} branco ${brancoMv} ruído ${ruido}: ${fora.map((x) => x.limiar)}`); break; }
  if (t === 29) confere(true, "30 combinações aleatórias de preto, branco e ruído: limiar sempre entre os dois");
}

console.log(falhas ? `\nERRO: ${falhas} falha(s)` : "\nOK: 0 falha(s)");
process.exit(falhas ? 1 : 0);
