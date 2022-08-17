class FoilUnsteadywake {
  final int n, m;
  float dt = 0, t = 0, dAoA, dA, dxf , omega, chord = 1.0, period, dfrac=0.5;
  float AoF, v2, pitch=0, p=0;
  float F;
  float b=50; //note b/resolution is true time constant...
  int resolution;

  boolean upstroke = false;

  BodyUnion body;
  NACA foil; 
  BDIM flow; 
  FloodPlot flood, flood2; 
  Window window;
  ReadData reader;
  PVector force;

  FoilUnsteadywake( int resolution, int xLengths, int yLengths, float dtReal, int Re, boolean QUICK, boolean isR) {
    this.resolution = resolution;
    n = xLengths*resolution;
    m = yLengths*resolution;
    window = new Window(n, m);

    body = new BodyUnion( new NACA(n/4*3-40, m/2, resolution*chord, .16, window), new CircleBody(n/4*3,m/2,1.0*resolution,window));
    //foil = new NACA(n/4*3, m/2, resolution*chord, .15, window); // NACA0012
    body.bodyList.get(0).rotate(-body.bodyList.get(0).phi+PI);
    body.bodyList.get(0).rotate(0);
    
    this.dt = dtReal*this.resolution;
    flow = new BDIM(n, m, dt, body, (float)resolution/Re, QUICK, -1); // flow is from right to left, which accords to the right-hand coord
    if(isR){
      flow.resume("./saved/init.bdim");
    }
    
    
    flood = new FloodPlot(window);
    flood.range = new Scale(-0.5, 0.5);
    flood.setLegend("vorticity");
    flood.setColorMode(1); 
    body.setColor(#CCCCCC);
  }
  
  void setFlapParams(float dxf, float dAoA, float dA) {
    this.dAoA = dAoA; 
    this.dA = dA; 
    this.dxf = dxf;
  }

  void computeState(float t) {
    AoF = atan2(0., 1.);
    v2 = 1;
    PVector pforce = body.bodyList.get(0).pressForce(flow.p);
    F = pforce.y*cos(AoF)+pforce.x*sin(AoF);
  }

  
  void update2(float AoA,float A,float Xf){
    if (flow.QUICK) {
      dt = flow.checkCFL();
      flow.dt = dt;
    }
    //flow.dt = this.dt;
    
    body.bodyList.get(0).rotate(AoA*flow.dt);
     body.bodyList.get(0).translate(Xf*flow.dt,A*flow.dt);
    flow.update(body);flow.update2();
    t += dt/resolution;  //nonedimension
    
    force = body.bodyList.get(0).pressForce(flow.p);
    
    //print("t="+nfs(t,2,3)+";  ");
    //print("drag="+nfs(force.x*2/this.resolution, 2, 2)+";  ");
    //print("lift="+nfs(force.y*2/this.resolution, 2, 2)+";  ");
    //println("AoA: "+(pitch-AoF)*180/PI);
  }
  
  void update() {
    if (flow.QUICK) {
      dt = flow.checkCFL();
      flow.dt = dt;
    }

    computeState(t);
    
    body.bodyList.get(0).rotate(-body.bodyList.get(0).phi-pitch+PI);
    //println("AoA: "+(pitch-AoF)*180/PI);

    flow.update(body);flow.update2();
    t += dt;
    
    print("t="+nfs(t/resolution,2,2)+";  ");
  }
  
  void display() {
    flood.display(flow.u.curl());
    body.display();
    //foil.displayVector(foil.pressForce(flow.p));
  }
}
