import p from 'puppeteer-core';
const url=process.argv[2]||'http://localhost:8768/index.html';
const D=JSON.parse(process.argv[3]||'[0.85,0.2,0.5]');
const b=await p.launch({executablePath:'/usr/bin/google-chrome',args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const pg=await b.newPage();pg.on('pageerror',e=>console.log('E',e.message));pg.on('console',m=>console.log('C',m.text()));
await pg.setViewport({width:390,height:844,deviceScaleFactor:2});
await pg.goto(url,{waitUntil:'networkidle0',timeout:90000});await pg.waitForFunction('window.ready',{timeout:60000});
const sh=async(f)=>{await pg.evaluate(()=>snapFit());await new Promise(r=>setTimeout(r,1200));await pg.screenshot({path:'shots/'+f})};
await sh('01_full_arm.png');
await pg.evaluate(()=>setPose(withF([-0.6,-1.6,0,0,0],HP.shaka)));await sh('01b_full_arm_shaka.png');
await pg.evaluate((D)=>{setZoom(true);setDir(...D)},D);
for(const [k,f] of [['point','02_pointing.png'],['shaka','03_shaka.png'],['peace','04_peace.png'],['gu','05_fist.png']]){
  await pg.evaluate(k=>setPose(withF([-0.6,-1.6,0,0,0],HP[k])),k);await sh(f);}
await b.close();
