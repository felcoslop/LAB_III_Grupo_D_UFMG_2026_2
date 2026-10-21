// smoke_opcoes.mjs
// Teste no navegador (Chromium sem tela) contra o robô de mentira (robo_falso.mjs) do cartão "Opções do robô": modo autônomo,
// sensores de linha da frente (CAD ou de lado), 5 ou 7 ToF, contagem girando e Wi-Fi na luta. Cada seletor manda "ajuste NOME VALOR",
// o robô reenvia o cabeçalho (linha F, R e G) e o painel mostra o valor em uso; trocar a quantidade de ToF reinicia o robô, o fluxo
// cai e o painel reconecta sozinho com 5 ou 7 sensores na tabela. Fora de ESPERA os seletores ficam travados.
// Uso: node teste/painel/smoke_opcoes.mjs   (precisa de "playwright" ou "playwright-core"; CHROME_PATH aponta o Chrome)
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
async function ate(cond, ms = 6000) { const t = Date.now(); while (Date.now() - t < ms) { if (await cond()) return true; await espera(40); } return false; }
const texto = (pagina, sel) => pagina.textContent(sel);
// botão pressionado de um seletor (data-aj) e o valor dele
const marcado = (pagina, nome) => pagina.evaluate((n) => { const b = document.querySelector(`#cardAj .seg[data-aj=${n}] button[aria-pressed=true]`); return b ? +b.dataset.v : null; }, nome);
const clica = (pagina, nome, v) => pagina.click(`#cardAj .seg[data-aj=${nome}] button[data-v="${v}"]`);

const robo = await iniciaRoboFalso();
const navegador = await pw.chromium.launch(lancamento);
const pagina = await navegador.newPage({ viewport: { width: 1400, height: 1000 } });
const erros = [];
pagina.on("pageerror", (e) => erros.push(String(e)));
pagina.on("console", (m) => { if (m.type() === "error" && !/WebGL|GPU|GL_|Failed to load resource|net::ERR/.test(m.text())) erros.push(m.text()); });
await pagina.goto("file://" + painelHtml);

// sem conexão: tudo travado e o aviso pede a conexão
confere((await pagina.locator("#cardAj .seg button:disabled").count()) === 10, "sem conexão os 10 botões de opção ficam travados");
confere(/Conecte ao robô/.test(await texto(pagina, "#ajNota")) && (await texto(pagina, "#ajPill")) === "sem robô", "e o cartão pede a conexão");
confere(!/GEOMETRIA_BANCADA|QTD_TOF|AUTO_START/.test(await texto(pagina, "#tbCfg")), "o guia do config.h não lista mais AUTO_START, QTD_TOF nem GEOMETRIA_BANCADA");

await pagina.selectOption("#selCon", "wifi");
await pagina.fill("#ip", "127.0.0.1:" + robo.porta());
await pagina.click("#bConn");
confere(await ate(async () => (await texto(pagina, "#status")) === "Wi-Fi conectado"), "conecta pelo Wi-Fi");
confere(await ate(() => pagina.evaluate(() => S.cfg && S.cfg.opcoes && S.estado === 0)), "recebe a linha F com as opções e o estado ESPERA");
confere((await pagina.locator("#cardAj .seg button:disabled").count()) === 0 && (await texto(pagina, "#ajPill")) === "no robô", "conectado em ESPERA: os seletores liberam");
confere((await marcado(pagina, "auto")) === 0 && (await marcado(pagina, "linha")) === 0 && (await marcado(pagina, "tof")) === 5 &&
        (await marcado(pagina, "alinha")) === 0 && (await marcado(pagina, "wifi")) === 0, "valores de fábrica marcados: autônomo desligado, linha no CAD, 5 ToF, contagem parada, Wi-Fi desliga ao largar");
confere((await pagina.evaluate(() => S.geo.length)) === 5 && (await pagina.evaluate(() => S.bg[0].x + "," + S.bg[0].y)) === "60,40", "tabela com 5 ToF e BFE na posição do CAD");

// modo autônomo
robo.recebidos.length = 0;
await clica(pagina, "auto", 1);
confere(await ate(() => robo.recebidos.includes("ajuste auto 1")), "modo autônomo Ligado manda ajuste auto 1");
confere(await ate(async () => (await marcado(pagina, "auto")) === 1), "o robô confirma e o painel marca Ligado");
confere(await ate(async () => /Modo autônomo ligado: da próxima vez/.test(await texto(pagina, "#ajNota"))), "e explica que vale quando o robô for ligado de novo", (await texto(pagina, "#ajNota")).slice(0, 90));
confere(/Modo autônomo ligado/.test(await texto(pagina, "#term")), "o terminal avisa do modo autônomo ligado");
await clica(pagina, "auto", 0);
confere(await ate(async () => (await marcado(pagina, "auto")) === 0), "Desligado volta a marcar");

// sensores de linha da frente
await clica(pagina, "linha", 1);
confere(await ate(() => robo.recebidos.includes("ajuste linha 1")), "De lado (90°) manda ajuste linha 1");
confere(await ate(() => pagina.evaluate(() => S.bg[0].x === 77 && S.bg[0].y === 81 && S.bg[1].y === -81 && S.bg[2].x === -50)),
        "BFE vai para y positivo e BFD para y negativo, e os de trás não mudam");
confere((await marcado(pagina, "linha")) === 1, "painel marca De lado");
await clica(pagina, "linha", 0);
confere(await ate(() => pagina.evaluate(() => S.bg[0].x === 60 && S.bg[0].y === 40)), "Como no CAD devolve a posição do CAD");

// contagem girando e Wi-Fi na luta: avisam do regulamento
await clica(pagina, "alinha", 1);
confere(await ate(() => robo.recebidos.includes("ajuste alinha 1")) && await ate(async () => (await marcado(pagina, "alinha")) === 1), "contagem Gira até alinhar: manda e marca");
confere(/regulamento da CoRA/.test(await texto(pagina, "#ajNota")), "e lembra do regulamento da CoRA");
await clica(pagina, "wifi", 1);
confere(await ate(() => robo.recebidos.includes("ajuste wifi 1")) && await ate(async () => (await marcado(pagina, "wifi")) === 1), "Wi-Fi Fica ligado: manda e marca");
confere(/proíbe rádio durante a luta/.test(await texto(pagina, "#ajNota")), "e lembra que o regulamento proíbe rádio na luta");
await clica(pagina, "alinha", 0); await clica(pagina, "wifi", 0);
confere(await ate(async () => (await marcado(pagina, "alinha")) === 0 && (await marcado(pagina, "wifi")) === 0), "voltam a Parado e Desliga ao largar");

// firmware com USAR_WIFI 0: a opção de Wi-Fi na luta avisa que não existe rádio
robo.cfg = { wifi: 0 };
await clica(pagina, "wifi", 1);
confere(await ate(async () => /USAR_WIFI 0/.test(await texto(pagina, "#ajNota"))), "firmware com USAR_WIFI 0: o aviso diz que não existe rádio");
await clica(pagina, "wifi", 0);
robo.cfg = {};
await ate(async () => (await marcado(pagina, "wifi")) === 0);

// 7 sensores: o robô reinicia, o fluxo cai e o painel reconecta com 7 na tabela
robo.recebidos.length = 0;
const reiniciosAntes = robo.reinicios;
await clica(pagina, "tof", 7);
confere(await ate(() => robo.recebidos.includes("ajuste tof 7")), "7 sensores manda ajuste tof 7");
confere(await ate(() => robo.reinicios === reiniciosAntes + 1), "o robô avisa e reinicia (o fluxo cai)");
confere(await ate(async () => (await pagina.evaluate(() => S.cfg && S.cfg.tof === 7 && S.geo.length === 7 && conectado)), 12000), "o painel reconecta sozinho e passa a ter 7 ToF na tabela");
confere((await marcado(pagina, "tof")) === 7, "e marca 7 sensores");
confere(await ate(() => pagina.evaluate(() => S.v.length === 7)), "os quadros D de 7 sensores são lidos");
// de volta para 5: a tabela não pode guardar TE e TD
await clica(pagina, "tof", 5);
confere(await ate(async () => (await pagina.evaluate(() => S.cfg && S.cfg.tof === 5 && S.geo.length === 5 && conectado)), 12000), "5 sensores: reconecta e a tabela perde TE e TD");
confere(await ate(async () => (await marcado(pagina, "tof")) === 5), "e marca 5 sensores");
robo.recebidos.length = 0;
await clica(pagina, "tof", 5);
await espera(200);
confere(!robo.recebidos.some((c) => c.startsWith("ajuste tof")), "clicar no valor que já está em uso não manda nada");

// fora de ESPERA os seletores travam
robo.estado = 2;
confere(await ate(() => pagina.evaluate(() => S.estado === 2)), "robô em BUSCA");
confere(await ate(async () => (await pagina.locator("#cardAj .seg button:disabled").count()) === 10), "em luta os 10 botões travam");
confere(await ate(async () => /só mudam em ESPERA/.test(await texto(pagina, "#ajNota"))), "e a nota manda enviar stop", (await texto(pagina, "#ajNota")).slice(0, 80));
robo.estado = 0;
confere(await ate(async () => (await pagina.locator("#cardAj .seg button:disabled").count()) === 0), "de volta em ESPERA os seletores liberam");

// padrão de fábrica
await clica(pagina, "auto", 1); await clica(pagina, "linha", 1);
await ate(async () => (await marcado(pagina, "auto")) === 1 && (await marcado(pagina, "linha")) === 1);
robo.recebidos.length = 0;
await pagina.click("#ajPadrao");
confere(await ate(() => robo.recebidos.includes("ajuste padrao")), "Voltar ao padrão de fábrica manda ajuste padrao");
confere(await ate(async () => (await marcado(pagina, "auto")) === 0 && (await marcado(pagina, "linha")) === 0 && (await marcado(pagina, "tof")) === 5), "e o painel marca os valores de fábrica");

// desconectar trava de novo
await pagina.click("#bDisc");
confere(await ate(async () => (await pagina.locator("#cardAj .seg button:disabled").count()) === 10 && (await texto(pagina, "#ajPill")) === "sem robô"), "desconectado: os seletores travam outra vez");

confere(erros.length === 0, "nenhum erro de JavaScript na página", erros.slice(0, 3).join(" | "));
await navegador.close();
robo.fecha();
console.log(falhas ? `\nERRO: ${falhas} falha(s)` : "\nOK: 0 falha(s)");
process.exit(falhas ? 1 : 0);
