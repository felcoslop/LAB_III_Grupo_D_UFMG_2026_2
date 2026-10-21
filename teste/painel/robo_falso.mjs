// robo_falso.mjs
// Servidor HTTP que imita o ESP32 com USAR_WIFI 1 (mesmos caminhos do telemetria.cpp: /ping, /stream em
// Server-Sent Events e /cmd?c=...), para testar o painel no navegador sem o robô. Quando recebe "s1",
// responde com o cabeçalho do protocolo (G, I, C, R, B, E, K, X, F) e passa a mandar os quadros D, Q e A.
// Cada comando recebido fica em "recebidos". O firmware de verdade é testado em teste/unidade/teste_wifi.cpp;
// aqui o que se confere é o lado do navegador.
import http from "node:http";

export function iniciaRoboFalso(porta = 0) {
  const recebidos = [];
  const fluxos = new Set();
  let streamLigado = false, t0 = Date.now(), timer = null;
  const cors = {"Access-Control-Allow-Origin": "*", "Access-Control-Allow-Private-Network": "true", "Cache-Control": "no-cache"};
  const manda = (l) => { for (const r of fluxos) r.write(`data: ${l}\n\n`); };

  const cabecalho = (cfg = {}) => {
    const f = Object.assign({motores: 1, wifi: 1, auto: 0, borda: 1, tof: 5, bancada: 1, radio: 1}, cfg);
    const sens = [["FE", 60, 40, 17], ["FC", 70, 0, 0], ["FD", 60, -40, -17], ["LE", 10, 60, 90], ["LD", 10, -60, -90]];
    sens.forEach(([n, x, y, a], i) => { manda(`G,${i},${n},${x},${y},${a}`); manda(`I,${i},1`); manda(`C,${i},1.0000,0.00,0`); });
    [["BFE", 60, 40], ["BFD", 60, -40], ["BTE", -50, 40], ["BTD", -50, -40]].forEach(([n, x, y], i) => manda(`R,${i},${n},${x},${y},2.7,600,3000,450,0,1`));
    manda("B,100.00,100.00,60.00,60.00,40.0,30.0,BANCADA");
    manda("E,ESPERA,CONTAGEM,BUSCA,MIRA,ATAQUE,FLANCO,ATAQUE_RE,DESTRAVA,RECUO,GIRO_BORDA,AVANCO_BORDA,SAIDA_LATERAL,LEQUE");
    manda("K,600,0,1,2,3,4,0,0,0,0");
    manda("X,1,152,550,0.25,80,200");
    manda(`F,${f.motores},${f.wifi},${f.auto},${f.borda},${f.tof},${f.bancada},${f.radio}`);
  };
  const quadro = () => {
    const ms = Date.now() - t0;
    manda(`D,300,310,-1,-1,-1,300,310,-1,-1,-1,1,150,300,2,3,290,80,3,1,0,31,${ms},20,90,100,2,150,0`);
    manda("Q,0,0,3000,3000,3000,3000");
    manda("A,0,0,0,1,1,0,0,20,0,0,0,0,0,0");
  };

  const srv = http.createServer((req, res) => {
    const u = new URL(req.url, "http://x");
    if (req.method === "OPTIONS") { res.writeHead(204, {...cors, "Access-Control-Allow-Methods": "GET, OPTIONS", "Access-Control-Allow-Headers": "*"}); res.end(); return; }
    if (u.pathname === "/ping") { res.writeHead(200, {...cors, "Content-Type": "text/plain"}); res.end("pong"); return; }
    if (u.pathname === "/cmd") {
      const c = u.searchParams.get("c") ?? "";
      recebidos.push(c);
      if (c === "s1") { streamLigado = true; cabecalho(servidor.cfg); }
      if (c === "s0") streamLigado = false;
      res.writeHead(200, {...cors, "Content-Type": "text/plain"}); res.end("ok"); return;
    }
    if (u.pathname === "/stream") {
      res.writeHead(200, {...cors, "Content-Type": "text/event-stream", Connection: "keep-alive"});
      res.write("retry: 1000\n\n");
      fluxos.add(res);
      req.on("close", () => fluxos.delete(res));
      return;
    }
    res.writeHead(404, cors); res.end("nao existe");
  });
  const servidor = {
    recebidos, cfg: {}, srv,
    derrubaFluxos() { for (const r of fluxos) r.destroy(); fluxos.clear(); },
    fluxosAbertos: () => fluxos.size,
    manda,
    porta: () => srv.address().port,
    fecha() { clearInterval(timer); servidor.derrubaFluxos(); srv.close(); srv.closeAllConnections?.(); },
  };
  timer = setInterval(() => { if (streamLigado && fluxos.size) quadro(); }, 20);
  return new Promise((ok) => srv.listen(porta, "127.0.0.1", () => ok(servidor)));
}
