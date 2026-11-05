import p from 'puppeteer-core';
const url=process.argv[2]||'http://localhost:8767/index.html', pre=process.argv[3]||'';
const b=await p.launch({executablePath:'/usr/bin/google-chrome',args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const pg=await b.newPage();pg.on('pageerror',e=>console.log('E',e.message));pg.on('console',m=>console.log('C',m.text()));
await pg.setViewport({width:390,height:844,deviceScaleFactor:2});
await pg.goto(url,{waitUntil:'networkidle0',timeout:90000});await pg.waitForFunction('window.ready',{timeout:60000});
const sh=async(f)=>{await pg.evaluate(()=>snapFit());await new Promise(r=>setTimeout(r,1200));await pg.screenshot({path:'shots/'+pre+f})};
await sh('01_full_arm.png');
if(!pre){await pg.evaluate(()=>demoAt(3.0));await sh('02_wave.png');
await pg.evaluate(()=>setPose([-0.6,-1.6,0,1.4,0]));await sh('03_fist.png');
await pg.evaluate(()=>setPose([-0.6,-1.6,0,0,1.57]));await sh('04_twist90.png');
await pg.evaluate(()=>setPose([-0.6,-1.6,0,1.4,1.57]));await sh('05_twist90_fist.png');
await pg.evaluate(()=>setPose([-0.6,-1.6,0,0,-1.57]));await sh('06_twist-90.png');}
await b.close();
