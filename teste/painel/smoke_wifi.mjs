// smoke_wifi.mjs
// Teste do painel no navegador (Chromium sem tela) contra o robô de mentira (robo_falso.mjs): o painel
// aberto do disco (file://) escolhe Wi-Fi, acha o robô, recebe o cabeçalho e os quadros, manda comandos
// em ordem, mostra a linha F do config.h e reconecta sozinho quando o fluxo cai.
// Uso: node teste/painel/smoke_wifi.mjs      (precisa do pacote "playwright" ou "playwright-core" do npm;
//      CHROME_PATH aponta o Chromium/Chrome quando ele não é o do Playwright)
import path from "node:path";
import { fileURLToPath } from "node:url";
import { createRequire } from "node:module";
import { iniciaRoboFalso } from "./robo_falso.mjs";

const aqui = path.dirname(fileURLToPath(import.meta.url));
const painelHtml = path.resolve(aqui, "..", "..", "painel", "painel_sumo.html");
const requer = createRequire(import.meta.url);      // require respeita NODE_PATH, o import de módulo não
let pw;
try { pw = requer("playwright"); } catch { pw = requer("playwright-core"); }
const lancamento = { headless: true, args: ["--use-gl=angle", "--use-angle=swiftshader", "--enable-unsafe-swiftshader", "--no-sandbox"] };
if (process.env.CHROME_PATH) lancamento.executablePath = process.env.CHROME_PATH;

let falhas = 0;
const confere = (ok, oque, extra = "") => { console.log(`${ok ? "ok   " : "FALHA"}  ${oque}${extra ? "  " + extra : ""}`); if (!ok) falhas++; };
const espera = (ms) => new Promise((r) => setTimeout(r, ms));
async function ate(cond, ms = 5000) { const t = Date.now(); while (Date.now() - t < ms) { if (await cond()) return true; await espera(50); } return false; }

const robo = await iniciaRoboFalso();
const navegador = await pw.chromium.launch(lancamento);
const pagina = await navegador.newPage();
const erros = [];
pagina.on("pageerror", (e) => erros.push(String(e)));
// erros de rede esperados (porta recusada de propósito, fluxo derrubado) e avisos do WebGL por software não contam
pagina.on("console", (m) => { if (m.type() === "error" && !/WebGL|GPU|GL_|Failed to load resource|net::ERR/.test(m.text())) erros.push(m.text()); });
await pagina.goto("file://" + painelHtml);

// escolha de Wi-Fi mostra o campo do endereço
confere(!(await pagina.isVisible("#ip")), "campo do endereço escondido na USB");
await pagina.selectOption("#selCon", "wifi");
confere(await pagina.isVisible("#ip"), "campo do endereço aparece no Wi-Fi");

// endereço errado: não conecta e explica
await pagina.fill("#ip", "127.0.0.1:9");
await pagina.click("#bConn");
confere(await ate(async () => /não respondeu/.test(await pagina.textContent("#term")), 6000), "robô fora do ar: o painel explica o que conferir");
confere((await pagina.textContent("#status")) === "desconectado", "robô fora do ar: continua desconectado");

// endereço certo
await pagina.fill("#ip", "127.0.0.1:" + robo.porta());
await pagina.click("#bConn");
confere(await ate(async () => (await pagina.textContent("#status")) === "Wi-Fi conectado"), "conecta pelo Wi-Fi");
confere(await ate(() => robo.recebidos.includes("s1")), "pede o cabeçalho (s1) assim que o fluxo abre");
confere(await ate(() => pagina.evaluate(() => S.geo.length === 5 && S.bg.length === 4)), "recebe a geometria dos 5 ToF e dos 4 sensores de borda");
confere(await ate(() => pagina.evaluate(() => S.tgt && S.tgt.visto === true && S.tgt.ang === 15)), "os quadros D chegam e viram oponente a 15 graus");
confere(await ate(async () => /^Wi-Fi · \d+ Hz/.test(await pagina.textContent("#pRede"))), "o indicador de dados mostra Wi-Fi e a taxa");
const hz = await pagina.evaluate(() => rede.hz);
confere(hz > 20, "taxa de quadros perto de 50 Hz", hz.toFixed(0) + " Hz");
confere(await pagina.evaluate(() => S.cfg && S.cfg.motores && S.cfg.wifi && !S.cfg.auto && S.cfg.tof === 5), "a linha F (config.h do firmware) é lida");
confere(await pagina.isDisabled("#selCon"), "a escolha USB ou Wi-Fi trava enquanto conectado");

// comandos chegam inteiros e na ordem
robo.recebidos.length = 0;
await pagina.evaluate(async () => { await enviar("mov 100 0 250 1"); await enviar("stop"); await enviar("lista"); });
confere(await ate(() => robo.recebidos.length >= 3), "três comandos chegam ao robô");
confere(JSON.stringify(robo.recebidos) === JSON.stringify(["mov 100 0 250 1", "stop", "lista"]), "comandos em ordem e sem alteração", JSON.stringify(robo.recebidos));
robo.recebidos.length = 0;
await pagina.evaluate(async () => { await enviar("go"); });
confere(await ate(() => robo.recebidos.length === 1), "ENTER vira um /cmd vazio ou o comando digitado");
// rajada de mov: só o último fica pendente, e o stop final nunca se perde
robo.recebidos.length = 0;
await pagina.evaluate(() => { for (let i = 1; i <= 30; i++) enviar(`mov ${i} 0 250`, true); enviar("stop", true); });
confere(await ate(() => robo.recebidos[robo.recebidos.length - 1] === "stop"), "rajada de mov termina com o stop entregue", `${robo.recebidos.length} comandos`);
confere(robo.recebidos.length <= 31, "rajada de mov não acumula fila alem do enviado");

// o fluxo cai e o navegador reconecta; o painel pede o cabeçalho de novo
const s1Antes = robo.recebidos.filter((c) => c === "s1").length;
robo.derrubaFluxos();
confere(await ate(async () => /reconectando/.test(await pagina.textContent("#status")), 3000), "fluxo derrubado: o painel mostra que está reconectando");
confere(await ate(async () => (await pagina.textContent("#status")) === "Wi-Fi conectado", 6000), "o navegador reconecta sozinho");
confere(await ate(() => robo.recebidos.filter((c) => c === "s1").length === s1Antes + 1), "depois de reconectar pede s1 de novo");

// desconectar manda s0 e solta a escolha
robo.recebidos.length = 0;
await pagina.click("#bDisc");
confere(await ate(() => robo.recebidos.includes("s0")), "Desconectar manda s0");
confere(await ate(async () => (await pagina.textContent("#status")) === "desconectado"), "volta a desconectado");
confere(!(await pagina.isDisabled("#selCon")), "a escolha USB ou Wi-Fi destrava");

confere(erros.length === 0, "nenhum erro de JavaScript na página", erros.slice(0, 3).join(" | "));
await navegador.close();
robo.fecha();
console.log(falhas ? `\nERRO: ${falhas} falha(s)` : "\nOK: 0 falha(s)");
process.exit(falhas ? 1 : 0);
