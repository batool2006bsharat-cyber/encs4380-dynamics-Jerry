
"use strict";

const $ = id => document.getElementById(id);
const DT = 0.002;
const DURATION = 18;
const U_MIN = 0;
const U_MAX = 20;
const SETPOINT = 1;
const SPEC = {overshoot:10, settling:3, sse:1};
const INITIAL_GAINS = {kp:2, ki:0, kd:0};
const palette = {setpoint:"#ffd36b", output:"#43dbc1", control:"#729dff", limit:"#ff7b8d"};

let plant = null;
let round = 1;
let attempts = 0;
let bestRoundScore = null;
let completedRounds = 0;
let completedPoints = 0;
let lastResult = null;
let revealed = false;

function random(min,max) { return min + Math.random()*(max-min); }
function pick(min,max) { return Number(random(min,max).toFixed(3)); }
function nextPlant() {
  const first = Math.random() < 0.5;
  return first
    ? {kind:"first", K:pick(0.8,1.4), tau:pick(0.65,1.6)}
    : {kind:"second", K:pick(0.75,1.3), zeta:pick(0.35,1.25), wn:pick(1.6,3.6)};
}
function gainValues() {
  return {kp:Number($("kp").value), ki:Number($("ki").value), kd:Number($("kd").value)};
}
function updateGains() {
  for (const name of ["kp","ki","kd"]) {
    $(name+"Value").textContent = Number($(name).value).toFixed(2);
  }
  if (lastResult) $("trialLabel").textContent = "Gains changed — test to update";
}
function resetGains() {
  for (const name of ["kp","ki","kd"]) $(name).value = INITIAL_GAINS[name];
  updateGains();
}

function simulate(p, g) {
  const count = Math.round(DURATION/DT);
  const time = new Array(count+1);
  const output = new Array(count+1);
  const control = new Array(count+1);
  let y=0, velocity=0, integral=0, previousY=0;
  for (let i=0; i<=count; i++) {
    const e = SETPOINT-y;
    const dydt = i===0 ? 0 : (y-previousY)/DT;
    const pd = g.kp*e-g.kd*dydt;
    const candidateI = integral+e*DT;
    const proposedU = pd+g.ki*candidateI;
    if (g.ki>0 && ((proposedU>=U_MIN && proposedU<=U_MAX) ||
        (proposedU>U_MAX && e<0) || (proposedU<U_MIN && e>0))) {
      integral=candidateI;
    }
    const u=Math.min(U_MAX,Math.max(U_MIN,pd+g.ki*integral));
    time[i]=i*DT;
    output[i]=y;
    control[i]=u;
    previousY=y;
    if (i===count) break;
    if (p.kind==="first") {
      y+=DT*(p.K*u-y)/p.tau;
    } else {
      const acceleration=p.K*p.wn*p.wn*u-2*p.zeta*p.wn*velocity-p.wn*p.wn*y;
      velocity+=DT*acceleration;
      y+=DT*velocity;
    }
    if (!Number.isFinite(y) || Math.abs(y)>1e6 || !Number.isFinite(velocity)) {
      throw Error("This design makes the simulation diverge. Reduce the gains and try again.");
    }
  }
  return {time,output,control};
}

function metrics(run) {
  const {time,output}=run;
  let peak=-Infinity, lastOutside=-1;
  for (let i=0;i<output.length;i++) {
    const y=output[i];
    if (y>peak) peak=y;
    if (Math.abs(y-SETPOINT)>0.02*SETPOINT) lastOutside=i;
  }
  const tail=output.slice(Math.floor(output.length*0.95));
  const finalMean=tail.reduce((a,b)=>a+b,0)/tail.length;
  const maxTail=Math.max(...tail), minTail=Math.min(...tail);
  const tailSteady=(maxTail-minTail)<=0.02*SETPOINT;
  return {
    overshoot:Math.max(0,100*(peak-SETPOINT)/SETPOINT),
    settling:lastOutside===output.length-1 ? null : (lastOutside<0 ? 0 : time[lastOutside+1]),
    sse:100*Math.abs(SETPOINT-finalMean)/SETPOINT,
    tailSteady,
    finalMean
  };
}
function evaluate(m) {
  const passOS=m.overshoot<=SPEC.overshoot+1e-9;
  const passTs=m.settling!==null && m.settling<=SPEC.settling;
  const passSse=m.tailSteady && m.sse<=SPEC.sse;
  const osPoints=passOS ? 40 : Math.max(0,40-1.5*(m.overshoot-SPEC.overshoot));
  const tsPoints=passTs ? 35 : m.settling===null ? 0 : Math.max(0,35-8*(m.settling-SPEC.settling));
  const ssePoints=passSse ? 25 : m.tailSteady ? Math.max(0,25-2.5*(m.sse-SPEC.sse)) : 0;
  return {passOS,passTs,passSse,allPass:passOS&&passTs&&passSse,
    osPoints,tsPoints,ssePoints,score:osPoints+tsPoints+ssePoints};
}
function updateScoreboard() {
  $("roundNumber").textContent=String(round);
  $("bestScore").textContent=bestRoundScore===null ? "—" : bestRoundScore.toFixed(1)+" / 100";
  const played=completedRounds+(bestRoundScore===null ? 0 : 1);
  const score=completedPoints+(bestRoundScore===null ? 0 : bestRoundScore);
  $("sessionScore").textContent=score.toFixed(1)+" / "+(played*100);
}

function plot(canvasId,t,series,caption,bounds=null) {
  const canvas=$(canvasId);
  const rect=canvas.getBoundingClientRect();
  const w=Math.max(310,rect.width),h=Math.max(230,rect.height);
  const dpr=Math.max(1,window.devicePixelRatio||1);
  canvas.width=Math.round(w*dpr); canvas.height=Math.round(h*dpr);
  const c=canvas.getContext("2d");
  c.setTransform(dpr,0,0,dpr,0,0);
  c.clearRect(0,0,w,h);
  const left=58,right=20,top=52,bottom=38;
  const pw=w-left-right,ph=h-top-bottom;
  let lower=Infinity,upper=-Infinity;
  for (const s of series) for (const v of s.values) {
    if (Number.isFinite(v)) { lower=Math.min(lower,v); upper=Math.max(upper,v); }
  }
  if (bounds) {lower=Math.min(lower,bounds[0]);upper=Math.max(upper,bounds[1]);}
  let pad=Math.max(0.08,(upper-lower)*0.1);
  lower-=pad;upper+=pad;
  const x=i=>left+(t[i]/t[t.length-1])*pw;
  const y=v=>top+ph-(v-lower)/(upper-lower)*ph;
  c.fillStyle="#0c1526";c.fillRect(0,0,w,h);
  c.font="12px Segoe UI,Arial";
  c.fillStyle="#eef4ff";c.fillText(caption,left,24);
  c.strokeStyle="#2a3a53";c.lineWidth=1;
  for(let k=0;k<=4;k++) {
    const yy=top+ph*k/4;
    c.beginPath();c.moveTo(left,yy);c.lineTo(w-right,yy);c.stroke();
    c.fillStyle="#a8b8d1";c.fillText((upper-k*(upper-lower)/4).toFixed(2),6,yy+4);
  }
  for(let k=0;k<=4;k++) {
    const xx=left+pw*k/4;
    c.beginPath();c.moveTo(xx,top);c.lineTo(xx,top+ph);c.stroke();
    c.fillStyle="#a8b8d1";c.fillText((k*t[t.length-1]/4).toFixed(0)+"s",xx-10,top+ph+20);
  }
  c.save();c.beginPath();c.rect(left,top,pw,ph);c.clip();
  for (const s of series) {
    c.strokeStyle=s.color;c.lineWidth=s.width||2.4;
    c.setLineDash(s.dash||[]);
    c.beginPath();
    const stride=Math.max(1,Math.ceil(t.length/1800));
    for(let i=0;i<t.length;i+=stride) {
      const xx=x(i),yy=y(s.values[i]);
      if(i===0)c.moveTo(xx,yy);else c.lineTo(xx,yy);
    }
    const j=t.length-1;c.lineTo(x(j),y(s.values[j]));c.stroke();
  }
  c.restore();c.setLineDash([]);
  let labelX=left;
  for(const s of series) {
    c.fillStyle=s.color;c.fillRect(labelX,35,13,3);
    c.fillStyle="#eef4ff";c.font="11px Segoe UI,Arial";c.fillText(s.label,labelX+18,40);
    labelX+=18+c.measureText(s.label).width+22;
  }
}
function draw(run) {
  const {time,output,control}=run;
  plot("responseChart",time,[
    {label:"Setpoint",color:palette.setpoint,values:time.map(()=>SETPOINT),dash:[5,5]},
    {label:"True plant output",color:palette.output,values:output}
  ],"Closed-loop output versus setpoint",[0,SETPOINT]);
  plot("controlChart",time,[
    {label:"Applied u(t)",color:palette.control,values:control},
    {label:"Actuator max",color:palette.limit,values:time.map(()=>U_MAX),dash:[5,5]}
  ],"Control signal u(t), limited to [0, 20]",[0,U_MAX]);
}
function displayResult(m,v) {
  const items=[
    {title:"Overshoot",value:m.overshoot.toFixed(2)+"%",spec:"≤ 10%",pass:v.passOS,points:v.osPoints,maximum:40},
    {title:"Settling time",value:m.settling===null?"Not settled":m.settling.toFixed(3)+" s",spec:"≤ 3.0 s (±2%)",pass:v.passTs,points:v.tsPoints,maximum:35},
    {title:"Steady-state error",value:m.tailSteady?m.sse.toFixed(2)+"%":"Unsettled",spec:"≤ 1% numerical tolerance",pass:v.passSse,points:v.ssePoints,maximum:25}
  ];
  $("specResults").innerHTML=items.map(item=>
    `<div class="result-item ${item.pass?"good":"bad"}"><h3>${item.title}</h3>`+
    `<div class="metric-data">${item.value}</div><p>Required: ${item.spec}</p>`+
    `<span class="status-line">${item.pass?"✓ PASS":"✗ FAIL"}</span>`+
    `<span class="points">${item.points.toFixed(1)} / ${item.maximum} points</span></div>`
  ).join("");
  $("roundScore").textContent=v.score.toFixed(1)+" / 100";
  const verdict=$("overallVerdict");
  verdict.className="verdict-text "+(v.allPass?"success":"warning");
  verdict.textContent=v.allPass ? "All specifications passed. Your controller meets the challenge!" :
    "At least one specification failed. Adjust the PID gains and try again.";
}
function suggest(m,v) {
  if (!v.passSse) return "Try increasing Ki moderately. Integral action is needed to drive steady-state error toward zero. If it causes oscillation, reduce it again.";
  if (!v.passOS) return "Try raising Kd moderately or reducing Kp/Ki. Derivative action often adds damping and lowers overshoot.";
  if (!v.passTs) return "Try increasing Kp a little for a quicker response, then adjust Kd to keep overshoot within 10%.";
  return "All specifications passed. Try a new random plant to practice a different system.";
}
function testGains() {
  $("errorMessage").textContent="";
  const selected=document.querySelector('input[name="prediction"]:checked');
  if (!selected) { $("errorMessage").textContent="Select a prediction before testing your gains."; return; }
  try {
    const gains=gainValues();
    const run=simulate(plant,gains);
    const m=metrics(run),v=evaluate(m);
    attempts++;
    if (!revealed) bestRoundScore=Math.max(bestRoundScore===null?0:bestRoundScore,v.score);
    lastResult={run,m,v,gains};
    $("trialLabel").textContent="Trial "+attempts+(revealed?" · practice after reveal":"");
    $("placeholder").hidden=true;
    draw(run);
    displayResult(m,v);
    const correct=(selected.value==="pass")===v.allPass;
    $("predictionFeedback").textContent=(correct?"✓ Correct prediction. ":"✗ Prediction did not match. ")+
      (v.allPass?"All three conditions passed.":"One or more conditions failed.");
    $("predictionFeedback").style.color=correct?"var(--accent)":"var(--danger)";
    $("revealBtn").disabled=false;
    $("hintText").hidden=true;
    updateScoreboard();
  } catch(err) {
    $("errorMessage").textContent=err.message;
  }
}
function revealPlant() {
  if (!lastResult) return;
  revealed=true;
  $("revealBox").hidden=false;
  $("revealBox").innerHTML=plant.kind==="first"
    ? `<strong>First-order plant revealed:</strong> K = ${plant.K.toFixed(3)}, τ = ${plant.tau.toFixed(3)} s.<br>G(s) = ${plant.K.toFixed(3)} / (${plant.tau.toFixed(3)}s + 1).`
    : `<strong>Second-order plant revealed:</strong> K = ${plant.K.toFixed(3)}, ζ = ${plant.zeta.toFixed(3)}, ωn = ${plant.wn.toFixed(3)} rad/s.<br>G(s) = Kωn² / (s² + 2ζωn s + ωn²).`;
  $("revealBox").innerHTML+="<br>After reveal, testing remains available for practice but no longer changes the round's best score.";
  $("revealBtn").disabled=true;
}
function startRound() {
  plant=nextPlant();
  attempts=0;bestRoundScore=null;lastResult=null;revealed=false;
  $("plantTag").textContent=plant.kind==="first"?"First-order plant":"Second-order plant";
  $("modelEquation").textContent=plant.kind==="first"
    ? "G(s) = K / (τs + 1) · K and τ are hidden"
    : "G(s) = Kωn² / (s² + 2ζωn s + ωn²) · parameters are hidden";
  $("revealBox").hidden=true;
  $("revealBox").textContent="";
  $("revealBtn").disabled=true;
  $("specResults").innerHTML='<div class="empty-message">Test a set of PID gains to see your results.</div>';
  $("overallVerdict").textContent="";
  $("overallVerdict").className="verdict-text";
  $("roundScore").textContent="—";
  $("trialLabel").textContent="No tests yet";
  $("placeholder").hidden=false;
  $("hintText").hidden=true;
  $("errorMessage").textContent="";
  $("predictionFeedback").textContent="Select your prediction before testing.";
  $("predictionFeedback").style.color="";
  document.querySelectorAll('input[name="prediction"]').forEach(el=>el.checked=false);
  for(const id of ["responseChart","controlChart"]) {
    const canvas=$(id);const ctx=canvas.getContext("2d");
    ctx.clearRect(0,0,canvas.width,canvas.height);
  }
  resetGains();updateScoreboard();
}
function newRound() {
  if (bestRoundScore!==null) {completedRounds++;completedPoints+=bestRoundScore;}
  round++;
  startRound();
  window.scrollTo({top:0,behavior:"smooth"});
}
for(const id of ["kp","ki","kd"]) $(id).addEventListener("input",updateGains);
$("resetGains").addEventListener("click",resetGains);
$("testBtn").addEventListener("click",testGains);
$("hintBtn").addEventListener("click",()=>{
  $("hintText").textContent=lastResult?suggest(lastResult.m,lastResult.v):
    "Start with a moderate Kp and a nonzero Ki, then use Kd to reduce overshoot. A P-only controller usually leaves a steady-state error.";
  $("hintText").hidden=false;
});
$("revealBtn").addEventListener("click",revealPlant);
$("newPlantBtn").addEventListener("click",newRound);
window.addEventListener("resize",()=>{if(lastResult)draw(lastResult.run);});
startRound();
