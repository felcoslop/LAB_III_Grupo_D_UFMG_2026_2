// smoke_demo.mjs
// Teste no navegador (Chromium sem tela) contra o robô de mentira (robo_falso.mjs) dos três cartões da demonstração:
// "Alinhar com o oponente" (comando alinha e linha Z), "Controle remoto (WASD)" (teclado e botões, mov e pwm com
// os flags de linha ignorada e texto calado, parar ao soltar) e o guia da configuração (linha F do firmware).
// Uso: node teste/painel/smoke_demo.mjs   (precisa de "playwright" ou "playwright-core"; CHROME_PATH aponta o Chrome)
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

const robo = await iniciaRoboFalso();
const navegador = await pw.chromium.launch(lancamento);
const pagina = await navegador.newPage({ viewport: { width: 1400, height: 1000 } });
const erros = [];
pagina.on("pageerror", (e) => erros.push(String(e)));
pagina.on("console", (m) => { if (m.type() === "error" && !/WebGL|GPU|GL_|Failed to load resource|net::ERR/.test(m.text())) erros.push(m.text()); });
await pagina.goto("file://" + painelHtml);

// antes de conectar: nada acontece e o guia pede a conexão
confere(/Conecte ao robô/.test(await texto(pagina, "#tbCfg")), "guia da configuração pede a conexão antes de conectar");
confere(await pagina.isDisabled("#alLiga"), "Alinhar fica travado sem conexão");

await pagina.selectOption("#selCon", "wifi");
await pagina.fill("#ip", "127.0.0.1:" + robo.porta());
await pagina.click("#bConn");
confere(await ate(async () => (await texto(pagina, "#status")) === "Wi-Fi conectado"), "conecta pelo Wi-Fi");
confere(await ate(() => pagina.evaluate(() => S.cfg && S.estado === 0)), "recebe a linha F e o estado ESPERA");

// ---- guia da configuração ----
confere(await ate(async () => (await texto(pagina, "#tbCfg")).includes("USAR_MOTORES")), "guia lista as chaves do config.h");
confere((await pagina.locator("#tbCfg .no").count()) === 0 && /serve para esta situação/.test(await texto(pagina, "#cfgNota")), "demonstração com motores 1 e Wi-Fi 1: nada a mudar no config.h");
await pagina.check("input[name=cfgPerfil][value=comp]");
confere(/Modo autônomo/.test(await texto(pagina, "#cfgNota")) && /Competição autônoma/.test(await texto(pagina, "#cfgNota")), "competição autônoma: manda ligar o Modo autônomo nas Opções do robô", (await texto(pagina, "#cfgNota")).slice(0, 80));
await pagina.check("input[name=cfgPerfil][value=demo]");

// ---- alinhar com o oponente ----
confere(!(await pagina.isDisabled("#alLiga")), "Alinhar liberado depois de conectar");
confere((await texto(pagina, "#alStatus")) === "DESLIGADO", "status começa DESLIGADO");
robo.recebidos.length = 0;
await pagina.fill("#alSeg", "30");
await pagina.click("#alLiga");
confere(await ate(() => robo.recebidos.includes("alinha on 30")), "o botão manda alinha on 30");
confere(await ate(async () => /PROCURANDO/.test(await texto(pagina, "#alStatus"))), "fase 1: PROCURANDO O OPONENTE");
confere(await ate(async () => /VIRANDO · erro \+\d+,\d°/.test(await texto(pagina, "#alStatus"))), "fase 2: VIRANDO com o erro em graus");
confere(await ate(async () => /ALINHADO/.test(await texto(pagina, "#alStatus")), 4000), "fase 3: ALINHADO");
confere((await pagina.getAttribute("#alStatus", "class")).includes("ok") && (await texto(pagina, "#alPill")) === "alinhado", "status e etiqueta ficam verdes");
confere(/pulsos · \d+ s/.test(await texto(pagina, "#alPulsos")), "mostra pulsos e tempo restante", await texto(pagina, "#alPulsos"));
confere(await pagina.isVisible("#alMarca"), "a barra mostra o oponente visto pelo ToF (quadros D)");
robo.recebidos.length = 0;
await pagina.click("#alPara");
confere(await ate(() => robo.recebidos.includes("alinha off")), "Parar manda alinha off");
confere(await ate(async () => (await texto(pagina, "#alStatus")) === "DESLIGADO"), "volta a DESLIGADO");
// só em ESPERA
robo.estado = 4;
await ate(() => pagina.evaluate(() => S.estado === 4));
robo.recebidos.length = 0;
await pagina.click("#alLiga");
await espera(300);
confere(!robo.recebidos.some((c) => c.startsWith("alinha on")), "robô em luta (estado 4): o painel não manda alinha on");
confere(/ESPERA/.test(await texto(pagina, "#alAviso")), "e explica que só vale em ESPERA", (await texto(pagina, "#alAviso")).slice(0, 70));
robo.estado = 0;
await ate(() => pagina.evaluate(() => S.estado === 0));

// ---- WASD ----
const ultimos = (n) => robo.recebidos.slice(-n);
const temMov = (padrao) => robo.recebidos.some((c) => padrao.test(c));
await pagina.click("#wsLiga");
confere((await texto(pagina, "#wsPill")) === "teclado LIGADO", "Ligar o teclado");
robo.recebidos.length = 0;
await pagina.keyboard.down("w");
confere(await ate(() => robo.recebidos.filter((c) => c === "mov 150 0 250 3").length >= 2), "W: mov 150 0 250 3 repetido a cada 120 ms (linha ignorada, sem texto)", robo.recebidos.join("|"));
await pagina.keyboard.up("w");
confere(await ate(() => robo.recebidos.includes("mov 0 0 60 3")), "soltar W: parada silenciosa (mov 0 0 60 3)");
robo.recebidos.length = 0;
await pagina.keyboard.down("a"); await espera(300); await pagina.keyboard.up("a");
confere(temMov(/^mov 0 120 250 3$/), "A: gira para a esquerda (w positivo)", robo.recebidos.join("|"));
robo.recebidos.length = 0;
await pagina.keyboard.down("d"); await espera(300); await pagina.keyboard.up("d");
confere(temMov(/^mov 0 -120 250 3$/), "D: gira para a direita (w negativo)");
robo.recebidos.length = 0;
await pagina.keyboard.down("s"); await espera(300); await pagina.keyboard.up("s");
confere(temMov(/^mov -150 0 250 3$/), "S: ré");
robo.recebidos.length = 0;
await pagina.keyboard.down("w"); await pagina.keyboard.down("a"); await espera(300); await pagina.keyboard.up("a"); await pagina.keyboard.up("w");
confere(temMov(/^mov 150 72 250 3$/), "W + A: curva (giro 60% do ajustado)", robo.recebidos.join("|"));
// espaço e setas
robo.recebidos.length = 0;
await pagina.keyboard.down("ArrowUp"); await espera(200);
await pagina.keyboard.press(" ");
confere(await ate(() => robo.recebidos.includes("stop")), "espaço manda stop");
await pagina.keyboard.up("ArrowUp");
confere(temMov(/^mov 150 0 250 3$/), "seta para cima vale como W");
// sem ignorar a linha: só o flag 2
await pagina.uncheck("#wsIgn");
robo.recebidos.length = 0;
await pagina.keyboard.down("w"); await espera(300); await pagina.keyboard.up("w");
confere(temMov(/^mov 150 0 250 2$/) && !temMov(/ 3$/), "sem 'ignorar a linha': flag 2, a linha vale", robo.recebidos.join("|"));
await pagina.check("#wsIgn");
// inverter giro
await pagina.check("#wsInv");
robo.recebidos.length = 0;
await pagina.keyboard.down("a"); await espera(300); await pagina.keyboard.up("a");
confere(temMov(/^mov 0 -120 250 3$/), "inverter o giro troca A e D");
await pagina.uncheck("#wsInv");
// velocidade pelos controles
await pagina.evaluate(() => { const e = document.getElementById("wsV"); e.value = "300"; e.dispatchEvent(new Event("input")); });
robo.recebidos.length = 0;
await pagina.keyboard.down("w"); await espera(300); await pagina.keyboard.up("w");
confere(temMov(/^mov 300 0 250 3$/) && (await texto(pagina, "#wsVV")) === "300 mm/s", "controle deslizante de velocidade vale no comando");
// PWM direto
await pagina.click("#wsPwm");
robo.recebidos.length = 0;
await pagina.keyboard.down("w"); await espera(300); await pagina.keyboard.up("w");
confere(temMov(/^pwm 35 35 250 3$/), "modo PWM direto: pwm 35 35 250 3", robo.recebidos.join("|"));
robo.recebidos.length = 0;
await pagina.keyboard.down("a"); await espera(300); await pagina.keyboard.up("a");
confere(temMov(/^pwm -35 35 250 3$/), "PWM: giro no lugar para a esquerda");
await pagina.click("#wsCin");
// botões da tela (mouse ou celular)
robo.recebidos.length = 0;
const caixa = await pagina.locator('#wsSetas button[data-k="f"]').boundingBox();
await pagina.mouse.move(caixa.x + caixa.width / 2, caixa.y + caixa.height / 2);
await pagina.mouse.down(); await espera(350); await pagina.mouse.up();
confere(temMov(/^mov 300 0 250 3$/) && await ate(() => robo.recebidos.includes("mov 0 0 60 3")), "botão ↑ da tela: anda enquanto apertado e para ao soltar", robo.recebidos.join("|"));
await espera(200);
// digitar no terminal não dirige o robô
robo.recebidos.length = 0;
await pagina.click("#cmd"); await pagina.keyboard.type("wasd");
await espera(300);
confere(!temMov(/^mov /), "digitar no terminal (wasd) não manda movimento");
await pagina.fill("#cmd", "");
// perder o foco da janela para o robô
await pagina.evaluate(() => document.activeElement && document.activeElement.blur());
robo.recebidos.length = 0;
await pagina.keyboard.down("w"); await espera(250);
await pagina.evaluate(() => window.dispatchEvent(new Event("blur")));
confere(await ate(() => robo.recebidos.includes("stop")), "janela perde o foco: o painel manda stop");
await pagina.keyboard.up("w");
// desligar o teclado
await pagina.click("#wsLiga");
confere((await texto(pagina, "#wsPill")) === "teclado desligado", "Desligar o teclado");
robo.recebidos.length = 0;
await pagina.keyboard.down("w"); await espera(300); await pagina.keyboard.up("w");
confere(!temMov(/^mov /), "com o teclado desligado, W não faz nada");
// em luta o WASD não manda
await pagina.click("#wsLiga");
robo.estado = 5; await ate(() => pagina.evaluate(() => S.estado === 5));
robo.recebidos.length = 0;
await pagina.keyboard.down("w"); await espera(300); await pagina.keyboard.up("w");
confere(!temMov(/^mov /) && /luta/.test(await texto(pagina, "#term")), "robô em luta: o WASD não manda movimento e avisa no terminal");
robo.estado = 0; await ate(() => pagina.evaluate(() => S.estado === 0));

// ---- modo autônomo ligado e motores desligados: avisos na demonstração ----
robo.cfg = { auto: 1, motores: 0 };
robo.recebidos.length = 0;
robo.derrubaFluxos();
confere(await ate(async () => /Modo autônomo ligado/.test(await texto(pagina, "#alAviso")) && /USAR_MOTORES 0/.test(await texto(pagina, "#alAviso")), 8000), "modo autônomo ligado e USAR_MOTORES 0: o cartão de alinhar avisa dos dois");
confere(/mudar para 1/.test(await texto(pagina, "#tbCfg")), "e o guia diz o que mudar no config.h");
await pagina.click("#alLiga");
await espera(250);
confere(!robo.recebidos.some((c) => c.startsWith("alinha on")), "com USAR_MOTORES 0 o painel não manda alinhar");
robo.cfg = {};

// ---- desconectar com o alinhamento rodando ----
robo.derrubaFluxos();
await ate(async () => (await texto(pagina, "#status")) === "Wi-Fi conectado", 8000);
await ate(() => pagina.evaluate(() => S.cfg && S.cfg.motores));
await pagina.click("#alLiga");
await ate(async () => /PROCURANDO|VIRANDO|ALINHADO/.test(await texto(pagina, "#alStatus")));
robo.recebidos.length = 0;
await pagina.click("#bDisc");
confere(await ate(() => robo.recebidos.includes("alinha off") && robo.recebidos.includes("s0")), "desconectar com o alinhamento ligado manda alinha off e s0");

confere(erros.length === 0, "nenhum erro de JavaScript na página", erros.slice(0, 3).join(" | "));
await navegador.close();
robo.fecha();
console.log(falhas ? `\nERRO: ${falhas} falha(s)` : "\nOK: 0 falha(s)");
process.exit(falhas ? 1 : 0);
