// smoke_calibracao.mjs
// Teste do assistente de calibração de pista no navegador (Chromium sem tela) contra o robô de mentira
// (robo_falso.mjs, que manda as linhas J): preto, dois passes de pares na faixa, tabela, histogramas, gravação
// por bordalim, aviso de sinal digital (D0), cancelar e desconectar no meio da calibração.
// Uso: node teste/painel/smoke_calibracao.mjs   (precisa de "playwright" ou "playwright-core"; CHROME_PATH aponta o Chrome)
import path from "node:path";
import { fileURLToPath } from "node:url";
import { createRequire } from "node:module";
import { iniciaRoboFalso } from "./robo_falso.mjs";

const aqui = path.dirname(fileURLToPath(import.meta.url));
const painelHtml = path.resolve(aqui, "..", "..", "painel", "painel_sumo.html");
const requer = createRequire(import.meta.url);
let pw;
try { pw = requer("playwright"); } catch { pw = requer("playwright-core"); }
const lancamento = { headless: true, args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--no-sandbox"] };
if (process.env.CHROME_PATH) lancamento.executablePath = process.env.CHROME_PATH;

let falhas = 0;
const confere = (ok, oque, extra = "") => { console.log(`${ok ? "ok   " : "FALHA"}  ${oque}${extra ? "  " + extra : ""}`); if (!ok) falhas++; };
const espera = (ms) => new Promise((r) => setTimeout(r, ms));
async function ate(cond, ms = 8000) { const t = Date.now(); while (Date.now() - t < ms) { if (await cond()) return true; await espera(50); } return false; }

const robo = await iniciaRoboFalso();
const navegador = await pw.chromium.launch(lancamento);
const pagina = await navegador.newPage({ viewport: { width: 1400, height: 1000 } });
const erros = [];
pagina.on("pageerror", (e) => erros.push(String(e)));
pagina.on("console", (m) => { if (m.type() === "error" && !/WebGL|GPU|GL_|Failed to load resource|net::ERR/.test(m.text())) erros.push(m.text()); });
await pagina.goto("file://" + painelHtml);
await pagina.selectOption("#selCon", "wifi");
await pagina.fill("#ip", "127.0.0.1:" + robo.porta());
await pagina.click("#bConn");
confere(await ate(async () => (await pagina.textContent("#status")) === "Wi-Fi conectado"), "conecta pelo Wi-Fi");
confere(await ate(() => pagina.evaluate(() => S.bg.length === 4 && S.estado === 0)), "recebe os 4 sensores de borda e o estado ESPERA");

// antes de iniciar, nada pode ser gravado
confere(await pagina.isDisabled("#calBtnPreto") && await pagina.isDisabled("#calBtnPasse") && await pagina.isDisabled("#calGrava"), "antes de iniciar, preto, passe e gravar ficam travados");

// 1. iniciar
await pagina.click("#calIni");
confere(await ate(() => robo.recebidos.includes("calpista on")), "Iniciar manda calpista on");
confere(await ate(async () => (await pagina.textContent("#calPill")) === "pronta"), "o estado vira pronta");
confere(await ate(async () => /janelas: \d+\/s/.test(await pagina.textContent("#calJ"))), "o painel mostra a taxa de janelas J recebidas");
confere(await pagina.isDisabled("#calBtnPasse"), "passe só libera depois do preto");

// 2. preto
robo.cena.faixa = [];
await pagina.click("#calBtnPreto");
confere(await ate(async () => (await pagina.textContent("#calPill")) === "gravando preto"), "gravando preto");
confere(await ate(() => pagina.evaluate(() => CAL.preto.length >= 120)), "juntou 120 janelas de preto");
await pagina.click("#calBtnPreto");
confere(await ate(() => pagina.evaluate(() => !CAL.gravando && CAL.preto.length >= 120)), "parar o preto fecha a gravação");
confere(!(await pagina.isDisabled("#calBtnPasse")), "passe liberado");

// 3. passe com os traseiros
await pagina.selectOption("#calPar", "tras");
robo.cena.faixa = ["BTE", "BTD"];
await pagina.click("#calBtnPasse");
confere(await ate(async () => (await pagina.textContent("#calPill")) === "gravando passe"), "gravando passe dos traseiros");
confere(await ate(() => pagina.evaluate(() => CAL.atual && CAL.atual.janelas.length >= 120)), "juntou 120 janelas do passe");
confere(await pagina.evaluate(() => CAL.vivo[2] > 100 && CAL.vivo[3] > 100 && !(CAL.vivo[0] > 0)), "o contador ao vivo conta a faixa só nos dois sensores do par");
await pagina.click("#calBtnPasse");
confere(await ate(() => pagina.evaluate(() => CAL.passes.length === 1 && CAL.res)), "fechar o passe calcula");
confere(await pagina.evaluate(() => CAL.res[2].ok && CAL.res[3].ok && !CAL.res[0].ok && !CAL.res[1].ok), "traseiros calculados, frontais ainda sem dados");
confere(await ate(async () => /nenhum passe/.test(await pagina.textContent("#calAvisos"))), "o aviso diz que falta passe nos frontais");

// 4. passe com os frontais
await pagina.selectOption("#calPar", "frente");
robo.cena.faixa = ["BFE", "BFD"];
await pagina.click("#calBtnPasse");
confere(await ate(() => pagina.evaluate(() => CAL.atual && CAL.atual.janelas.length >= 120)), "juntou 120 janelas do passe dos frontais");
await pagina.click("#calBtnPasse");
confere(await ate(() => pagina.evaluate(() => CAL.passes.length === 2 && CAL.res && CAL.res.every((r) => r.ok))), "os quatro sensores calculam");
const lims = await pagina.evaluate(() => CAL.res.map((r) => r.limiar));
confere(lims.every((l) => l > 500 && l < 2300), "limiares entre o branco (300) e o preto (2800)", lims.join(" "));
confere((await pagina.locator("#calHist svg").count()) === 4, "quatro histogramas desenhados");
confere((await pagina.locator("#tbCal tr").count()) === 4 && /boa/.test(await pagina.textContent("#tbCal")), "tabela com 4 linhas e nível boa");
confere(!(await pagina.isDisabled("#calGrava")) && /4 sensores/.test(await pagina.textContent("#calGrava")), "botão Gravar liberado para 4 sensores");

// fração mexe no limiar
const antes = lims[0];
await pagina.evaluate(() => { const e = document.getElementById("calFrac"); e.value = "70"; e.dispatchEvent(new Event("input")); });
const depois = await pagina.evaluate(() => CAL.res[0].limiar);
confere(depois > antes, "fração 70% sobe o limiar", `${antes} -> ${depois}`);
await pagina.evaluate(() => { const e = document.getElementById("calFrac"); e.value = "50"; e.dispatchEvent(new Event("input")); });

// 5. gravar
robo.recebidos.length = 0;
await pagina.click("#calGrava");
confere(await ate(() => robo.recebidos.filter((c) => c.startsWith("bordalim ")).length === 4), "mandou 4 comandos bordalim");
const bl = robo.recebidos.filter((c) => c.startsWith("bordalim "));
confere(bl.map((c) => c.split(" ")[1]).join(",") === "BFE,BFD,BTE,BTD", "na ordem dos sensores", bl.join(" | "));
confere(bl.every((c) => /^bordalim [A-Z]{3} \d+ \d+ \d+$/.test(c)), "cada comando com nome, limiar, preto e branco", bl[0]);
confere(await ate(() => robo.recebidos.includes("calpista off")), "depois de gravar desliga o calpista");
confere(await ate(async () => (await pagina.textContent("#calPill")) === "gravada"), "estado gravada");
confere(await ate(async () => (await pagina.textContent("#blm0")).includes("●")), "a tabela de borda marca o limiar vindo da pista (●)");
confere(await pagina.evaluate(() => S.bg.every((g) => g.pista && g.nvs)), "o painel leu o 11º campo do R (limiar de pista) dos quatro sensores");

// 6. sinal digital (fio no D0)
robo.perfil.digital = true;
await pagina.click("#calIni");
confere(await ate(async () => (await pagina.textContent("#calPill")) === "pronta"), "nova calibração começa do zero");
confere(await pagina.evaluate(() => CAL.preto.length === 0 && CAL.passes.length === 0 && !CAL.res), "e apaga os dados da anterior");
robo.cena.faixa = [];
await pagina.click("#calBtnPreto");
await ate(() => pagina.evaluate(() => CAL.preto.length >= 110));
await pagina.click("#calBtnPreto");
await pagina.selectOption("#calPar", "todos");
robo.cena.faixa = ["BFE", "BFD", "BTE", "BTD"];
await pagina.click("#calBtnPasse");
await ate(() => pagina.evaluate(() => CAL.atual && CAL.atual.janelas.length >= 110));
await pagina.click("#calBtnPasse");
confere(await ate(() => pagina.evaluate(() => CAL.res && CAL.res.every((r) => r.ok && r.digital))), "passe dos quatro juntos: sinal digital reconhecido nos quatro");
confere(/D0/.test(await pagina.textContent("#calAvisos")), "o aviso do D0 aparece no painel");
confere(!/boa/.test(await pagina.textContent("#tbCal")), "digital nunca sai como boa");

// 7. cancelar
robo.recebidos.length = 0;
await pagina.click("#calCancela");
confere(await ate(() => robo.recebidos.includes("calpista off")), "Cancelar manda calpista off");
confere((await pagina.textContent("#calPill")) === "parada" && (await pagina.isDisabled("#calGrava")), "volta a parada e trava a gravação");

// 8. desconectar no meio da gravação
robo.perfil.digital = false;
await pagina.click("#calIni");
await ate(async () => (await pagina.textContent("#calPill")) === "pronta");
await pagina.click("#calBtnPreto");
robo.recebidos.length = 0;
await pagina.click("#bDisc");
confere(await ate(() => robo.recebidos.includes("calpista off") && robo.recebidos.includes("s0")), "desconectar no meio da calibração manda calpista off e s0");
confere((await pagina.textContent("#calPill")) === "parada", "e a calibração volta a parada");

confere(erros.length === 0, "nenhum erro de JavaScript na página", erros.slice(0, 3).join(" | "));
await navegador.close();
robo.fecha();
console.log(falhas ? `\nERRO: ${falhas} falha(s)` : "\nOK: 0 falha(s)");
process.exit(falhas ? 1 : 0);
