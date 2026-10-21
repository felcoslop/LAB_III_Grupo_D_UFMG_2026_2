// robo_falso.mjs
// Servidor HTTP que imita o ESP32 com USAR_WIFI 1 (mesmos caminhos do telemetria.cpp: /ping, /stream em
// Server-Sent Events e /cmd?c=...), para testar o painel no navegador sem o robô. Quando recebe "s1",
// responde com o cabeçalho do protocolo (G, I, C, R, B, E, K, X, F) e passa a mandar os quadros D, Q e A.
// Cada comando recebido fica em "recebidos". O firmware de verdade é testado em teste/unidade/teste_wifi.cpp;
// aqui o que se confere é o lado do navegador.
// Calibração de pista: com "calpista on" o robô de mentira manda uma linha J (janela de 20 ms) por quadro, com as
// leituras da "cena" (servidor.cena) e do "perfil" (servidor.perfil); "bordalim NOME LIM PRETO BRANCO" muda o
// limiar e reenvia o cabeçalho, como o debug.cpp.
// Demonstração de alinhamento: "alinha on S" manda linhas Z (busca, virando, alinhado, em 0,7 s) e "alinha off" fecha com a
// fase 0, como o estados.cpp; "servidor.estado" é o estado da máquina que vai nos quadros D (0 é ESPERA).
// Wi-Fi leve: com "ajuste leve 1" (servidor.aj.leve) o robô de mentira corta o stream como o telemetria.cpp: D a cada 100 ms,
// Q e A a cada 250 ms; a Z e a J passam. O robô de mentira sai de fábrica com leve 0, para os testes de taxa do painel.
// Opções do painel: "ajuste NOME VALOR" (auto, linha, tof, alinha, wifi, leve) muda servidor.aj e reenvia o cabeçalho (linha F com as opções,
// linhas R com os sensores de linha da frente de lado ou no CAD, linhas G com 5 ou 7 ToF), como o debug.cpp; "ajuste tof" diferente do
// em uso avisa, derruba o fluxo e volta com a quantidade nova (o robô reiniciou); "ajuste padrao" volta ao valor de fábrica; fora de
// ESPERA recusa com "! pare o robo antes".
import http from "node:http";

export function iniciaRoboFalso(porta = 0) {
  const recebidos = [];
  const fluxos = new Set();
  let streamLigado = false, t0 = Date.now(), timer = null, pista = false, seqJ = 0, rng = 12345, alinha = null, tZ = 0;
  const nomes = ["BFE", "BFD", "BTE", "BTD"];
  const aj = {auto: 0, linha: 0, tof: 5, alinha: 0, wifi: 0, leve: 0};
  let ultD = 0, ultQA = 0, ultM = 0;
  const bordas = nomes.map((n) => ({lim: 600, preto: 3000, branco: 450, grav: 0, pista: 0}));
  const rand = () => { rng = (rng * 1664525 + 1013904223) >>> 0; return rng / 4294967296; };
  const gauss = () => { let s = 0; for (let k = 0; k < 6; k++) s += rand(); return (s - 3) / 0.7071; };
  const cors = {"Access-Control-Allow-Origin": "*", "Access-Control-Allow-Private-Network": "true", "Cache-Control": "no-cache"};
  const manda = (l) => { for (const r of fluxos) r.write(`data: ${l}\n\n`); };

  const cabecalho = (cfg = {}) => {
    const f = Object.assign({motores: 1, wifi: 1, auto: aj.auto, borda: 1, tof: aj.tof, bancada: 0, radio: 1, linha: aj.linha, alinhaC: aj.alinha, wifiLuta: aj.wifi, leve: aj.leve}, cfg);
    const sens = [["FE", 60, 40, 17], ["FC", 70, 0, 0], ["FD", 60, -40, -17], ["LE", 10, 60, 90], ["LD", 10, -60, -90], ["TE", -30, 66, 160], ["TD", -30, -66, -160]];
    sens.slice(0, f.tof).forEach(([n, x, y, a], i) => { manda(`G,${i},${n},${x},${y},${a}`); manda(`I,${i},1`); manda(`C,${i},1.0000,0.00,0`); });
    const frente = f.linha ? [77, 81] : [60, 40];
    [["BFE", frente[0], frente[1]], ["BFD", frente[0], -frente[1]], ["BTE", -50, 40], ["BTD", -50, -40]].forEach(([n, x, y], i) => {
      const b = bordas[i];
      manda(`R,${i},${n},${x},${y},2.7,${b.lim},${b.preto},${b.branco},${b.grav},${b.preto > b.branco ? 1 : 0},${b.pista}`);
    });
    manda("B,100.00,100.00,60.00,60.00,40.0,30.0,ROBO");
    manda("E,ESPERA,CONTAGEM,BUSCA,MIRA,ATAQUE,FLANCO,ATAQUE_RE,DESTRAVA,RECUO,GIRO_BORDA,AVANCO_BORDA,SAIDA_LATERAL,LEQUE");
    manda("K,600,0,1,2,3,4,0,0,0,0");
    manda("X,1,152,550,0.25,80,200");
    manda(`F,${f.motores},${f.wifi},${f.auto},${f.borda},${f.tof},${f.bancada},${f.radio},${f.linha},${f.alinhaC},${f.wifiLuta}${f.semLeve ? "" : "," + f.leve}`);   // cfg.semLeve imita o firmware sem o Wi-Fi leve (F com 10 campos)
  };
  // uma janela de 20 ms: 20 leituras por sensor, na cena atual. cena = {faixa: ["BTE", "BTD"]} põe esses sensores
  // sobre a faixa branca e os outros sobre o preto. perfil: níveis em mV, ruído, arranhões (probabilidade e nível),
  // módulo invertido (branco = tensão alta) e sinal digital (D0: só 0 V e 3,3 V).
  const janela = () => {
    const p = servidor.perfil, faixa = servidor.cena.faixa || [];
    const v = [];
    nomes.forEach((n) => {
      const naFaixa = faixa.includes(n);
      let mn = Infinity, mx = -Infinity, soma = 0;
      for (let k = 0; k < 20; k++) {
        let x;
        if (p.digital) x = naFaixa ? 0 : 3300;
        else {
          x = (naFaixa ? p.brancoMv : p.pretoMv) + gauss() * p.ruidoMv;
          if (!naFaixa && rand() < p.arranhaoProb / 20) x = p.arranhaoMv + gauss() * p.ruidoMv;   // arranhão claro no preto
          if (naFaixa && rand() < p.sujeiraProb / 20) x = p.sujeiraMv + gauss() * p.ruidoMv;      // pedaço escuro na faixa
          if (p.invertido) x = 4095 - x;
          x = Math.max(0, Math.min(3300, Math.round(x)));
        }
        mn = Math.min(mn, x); mx = Math.max(mx, x); soma += x;
      }
      v.push(mn, mx, Math.round(soma / 20));
    });
    return `J,${seqJ++},20,${v.join(",")}`;
  };
  const quadro = () => {
    const ms = Date.now() - t0;
    const mm = [300, -1, -1, -1, -1, -1, -1].slice(0, aj.tof), br = mm.map((x) => (x > 0 ? x + 10 : x));
    const agora = Date.now();
    if (!aj.leve || agora - ultD >= 100) {
      ultD = agora;
      manda(`D,${mm.join(",")},${br.join(",")},1,150,300,2,3,290,80,3,1,${servidor.estado},31,${ms},20,90,100,2,150,0`);
    }
    if (!aj.leve || agora - ultQA >= 250) {
      ultQA = agora;
      manda("Q,0,0,3000,3000,3000,3000");
      manda("A,0,0,0,1,1,0,0,20,0,0,0,0,0,0");
    }
    if (agora - ultM >= (aj.leve ? 500 : 250)) { ultM = agora; manda("M,4,2,100,100,01234567"); }    // mapa de probabilidade: 4 Hz no completo, 2 Hz no leve
    if (pista) manda(janela());
    if (alinha) {
      const dt = Date.now() - alinha.t0;
      if (dt >= alinha.fim) { manda("# alinha: tempo esgotado"); manda("Z,0,0,0," + alinha.pulsos + (servidor.cfg.zAntigo ? "" : ",0,0")); alinha = null; }
      else if (Date.now() - tZ >= 100) {
        tZ = Date.now();
        const fase = dt < 300 ? 1 : dt < 700 ? 2 : 3, erro = fase === 1 ? 0 : fase === 2 ? 125 - Math.round((dt - 300) * 0.25) : 12;
        alinha.pulsos = Math.floor(dt / 160);
        // giro desde o rumo inicial em décimos de grau (o oponente fica a 12,5 graus) e o limite de cada lado; firmware antigo manda só 4 campos
        const giro = fase === 1 ? 0 : fase === 2 ? Math.round(Math.min(125, (dt - 300) * 0.3125)) : 125;
        manda(`Z,${fase},${erro},${Math.max(0, Math.ceil((alinha.fim - dt) / 1000))},${alinha.pulsos}` + (servidor.cfg.zAntigo ? "" : `,${giro},90`));
      }
    }
  };

  // o robô grava a quantidade de ToF, avisa e reinicia: o fluxo cai e o painel (EventSource) volta a conectar e pede o cabeçalho (s1)
  const reiniciaRobo = (tof) => {
    manda("# tof gravado: o robo reinicia em 1 s para configurar os sensores");
    setTimeout(() => { aj.tof = tof; streamLigado = false; servidor.reinicios++; servidor.derrubaFluxos(); }, 300);
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
      const al = /^alinha on(?: (\d+))?$/.exec(c);
      if (al) {
        if (servidor.estado !== 0) manda("! alinha so funciona com o robo parado (ESPERA)");
        else { const s = +(al[1] || 60); alinha = {t0: Date.now(), fim: s * 1000, pulsos: 0}; manda(`# alinha: ligado por ${s} s (so gira no lugar, nao anda, linha ignorada, so acompanha o oponente da frente, no maximo 90 graus de cada lado); alinha off para parar`); }
      }
      if (c === "alinha off" && alinha) { alinha = null; manda("# alinha: desligado"); manda("Z,0,0,0,0" + (servidor.cfg.zAntigo ? "" : ",0,0")); }
      if (c === "stop" || c === "go") { if (alinha) { alinha = null; manda("Z,0,0,0,0" + (servidor.cfg.zAntigo ? "" : ",0,0")); } }
      const ajc = /^ajuste(?: (\w+)(?: (\d+))?)?$/.exec(c);
      if (ajc) {
        const mostra = () => manda(`# ajustes (gravados no robo): auto ${aj.auto} | linha ${aj.linha} | tof ${aj.tof} | alinha ${aj.alinha} | wifi ${aj.wifi} | leve ${aj.leve}`);
        if (!ajc[1]) mostra();
        else if (servidor.estado !== 0) manda("! pare o robo antes (stop): as opcoes so mudam em ESPERA");
        else if (ajc[1] === "padrao") {
          const reinicia = aj.tof !== 5;
          Object.assign(aj, {auto: 0, linha: 0, alinha: 0, wifi: 0, leve: 0});
          manda("# ajustes: valores de fabrica");
          if (reinicia) reiniciaRobo(5); else { mostra(); cabecalho(servidor.cfg); }
        } else if (!(ajc[1] in aj) || ajc[2] === undefined || (ajc[1] === "tof" ? ![5, 7].includes(+ajc[2]) : ![0, 1].includes(+ajc[2]))) {
          manda("! uso: ajuste auto|linha|alinha|wifi|leve 0|1 | ajuste tof 5|7 | ajuste padrao | ajuste (mostra)");
        } else if (ajc[1] === "tof" && +ajc[2] !== aj.tof) reiniciaRobo(+ajc[2]);
        else { aj[ajc[1]] = +ajc[2]; if (ajc[1] === "auto") manda(+ajc[2] ? "# modo autonomo gravado: vale quando o robo for ligado de novo (para largar agora, use go)" : "# modo autonomo desligado: o robo fica em ESPERA ate mandarem"); mostra(); cabecalho(servidor.cfg); }
      }
      if (c === "calpista on") { pista = true; manda("# calpista ON: linhas J com minimo, maximo e media de cada sensor de borda"); }
      if (c === "calpista off") { pista = false; manda("# calpista OFF"); }
      const bl = /^bordalim (\w+) (\d+)(?: (\d+) (\d+))?$/.exec(c);
      if (bl) {
        const i = nomes.indexOf(bl[1].toUpperCase());
        if (i >= 0) {
          const lim = +bl[2], p = bl[3] ? +bl[3] : lim * 3, b = bl[4] ? +bl[4] : Math.round(lim / 3);
          Object.assign(bordas[i], {lim, preto: p, branco: b, grav: 1, pista: 1});
          manda(`# ${bl[1]}: limiar de pista ${lim} mV gravado (preto ${p}, branco ${b})`);
          cabecalho(servidor.cfg);
        }
      }
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
    recebidos, cfg: {}, srv, bordas, estado: 0, aj, reinicios: 0,
    cena: {faixa: []},
    perfil: {pretoMv: 2800, brancoMv: 300, ruidoMv: 40, arranhaoProb: 0, arranhaoMv: 1700, sujeiraProb: 0, sujeiraMv: 900, invertido: false, digital: false},
    pistaLigada: () => pista,
    derrubaFluxos() { for (const r of fluxos) r.destroy(); fluxos.clear(); },
    fluxosAbertos: () => fluxos.size,
    manda,
    porta: () => srv.address().port,
    fecha() { clearInterval(timer); servidor.derrubaFluxos(); srv.close(); srv.closeAllConnections?.(); },
  };
  timer = setInterval(() => { if (streamLigado && fluxos.size) quadro(); }, 20);
  return new Promise((ok) => srv.listen(porta, "127.0.0.1", () => ok(servidor)));
}
