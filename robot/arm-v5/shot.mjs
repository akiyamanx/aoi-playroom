import p from 'puppeteer-core';
const url=process.argv[2]||'http://localhost:8769/index.html';
const b=await p.launch({executablePath:'/usr/bin/google-chrome',args:['--no-sandbox','--use-angle=swiftshader','--enable-unsafe-swiftshader']});
const pg=await b.newPage();pg.on('pageerror',e=>console.log('E',e.message));
await pg.setViewport({width:390,height:844,deviceScaleFactor:2});
await pg.goto(url,{waitUntil:'networkidle0',timeout:90000});await pg.waitForFunction('window.ready',{timeout:60000});
const sh=async(f)=>{await pg.evaluate(()=>snapFit());await new Promise(r=>setTimeout(r,1500));await pg.screenshot({path:'shots/'+f});console.log('shot',f)};
// 1 full arm neutral
await pg.evaluate(()=>{setBoth(false);setPose(new Array(10).fill(0));setZoom(false);setDir(0.85,0.2,0.5)});
await sh('01_full_arm.png');
// 1b full arm shaka
await pg.evaluate(()=>setPose(withF([-0.6,-1.6,0,0,0],HP.shaka)));
await sh('01b_full_arm_shaka.png');
// 2 hand close-up: side view showing yoke centering (look along Y roughly)
await pg.evaluate(()=>{setZoom(true);setDir(0.05,0.0,1.0);setPose(withF([-0.55,-1.55,0,0,0],HP.pa))});
await sh('02_hand_centered.png');
// 2b wrist flexed toward palm + fist to show curl direction
await pg.evaluate(()=>{setZoom(true);setDir(0.55,0.2,0.8);setPose(withF([-0.55,-1.55,0.7,0,0],HP.gu))});
await sh('02b_hand_flex_fist.png');
// 3 shaka close-up
await pg.evaluate(()=>{setZoom(true);setDir(0.7,0.25,0.65);setPose(withF([-0.6,-1.6,0,0,0],HP.shaka))});
await sh('03_shaka.png');
// 4 two-hand squeeze
await pg.evaluate(()=>{setZoom(false);setDir(0.2,0.25,1.0);setBoth(true)});
await new Promise(r=>setTimeout(r,2800));
await sh('04_two_hands.png');
await b.close();
