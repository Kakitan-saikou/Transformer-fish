class RotatingFoil {
  final int n, m;
  float dt = 0, t = 0, dAoA, uAoA,dA,uA, omega, phi,chord = 1.0, period, dfrac=0.5;
  float AoF, v2, pitch=0, Heave = 0,p=0;
  float F;
  float b=50; //note b/resolution is true time constant...
  int resolution;

  boolean upstroke = false;

  NACA foil; 
  BDIM flow; 
  FloodPlot flood, flood2; 
  Window window;
  ReadData reader;
  PVector force;

  RotatingFoil( int resolution, int xLengths, int yLengths, float dtReal, int Re, boolean QUICK) {
    this.resolution = resolution;
    n = xLengths*resolution;
    m = yLengths*resolution;
    window = new Window(n, m);

    foil = new NACA(n/2, m/2, resolution*chord, .16, window); // NACA0016
    foil.rotate(-foil.phi+PI);
    foil.rotate(0);
    
    this.dt = dtReal*this.resolution;
    flow = new BDIM(n, m, dt, foil, (float)resolution/Re, QUICK, -1); // flow is from right to left, which accords to the right-hand coord

    //flow.write("pitch\\saved\\init.bdim");
    flood = new FloodPlot(window);
    flood.range = new Scale(-0.5, 0.5);
    flood.setLegend("vorticity");
    flood.setColorMode(1); 
    foil.setColor(#CCCCCC);
  }
  
  void setFlapParams(float St, float dAoA, float uAoA, float dA, float uA, float phi ) {
    this.dAoA = dAoA; 
    this.uAoA = uAoA; 
    this.dA = dA; 
    this.uA = uA;
    this.phi = phi;
    //this.omega = TWO_PI/resolution * stru/(2*1.0*chord);
    //this.omega = TWO_PI * St/(1.0*chord);  orignal flapping freq 
    this.omega = TWO_PI * St/(1.0*0.16);
    this.period = TWO_PI/omega;
    //foil.translate(0,0);
  }

  void computeState(float t) {
    AoF = atan2(0., 1.);
    v2 = 1;
    PVector pforce = foil.pressForce(flow.p);
    F = pforce.y*cos(AoF)+pforce.x*sin(AoF);
  }

  float computePitch(float t) {
    float pitchAmp = dAoA;
    if (pitch<=0) {
      pitchAmp = uAoA;
    }
    return pitchAmp*sin(omega*t+phi)+AoF; // position control
    //return pitchAmp*omega*cos(omega*t); // vel control
  }
    float computeHeave(float t) {
    float HeaveAmp = dA;
    if (Heave<=0) {
      HeaveAmp = uA;
    }
    return HeaveAmp*sin(omega*t)+m/2.; // position control
    //return HeaveAmp*omega*cos(omega*t); // vel control
  }
  void update2(){
    flow.dt = this.dt;
    
    computeState(t);
    pitch = computePitch(t);
    //foil.rotate(0*flow.dt); // vel control
    foil.rotate(-foil.phi-pitch+PI); // position control
    Heave = computeHeave(t);
    //foil.translate(0.,3*flow.dt); // vel control
    foil.translate(0.,Heave - foil.xc.y); // position control

    flow.update(foil);flow.update2();
    t += dt/resolution;  //nonedimension
    force = foil.pressForce(flow.p);
    
    print("t="+nfs(t,2,3)+";  ");
    print("drag="+nfs(force.x*2/this.resolution, 2, 2)+";  ");
    print("lift="+nfs(force.y*2/this.resolution, 2, 2)+";  ");
    //print("dt="+nfs(0.3*flow.dt,2,3)+";  ");
    println("AoA: "+(pitch-AoF)*180/PI);
  }
  
  void update() {
    if (flow.QUICK) {
      dt = flow.checkCFL();
      flow.dt = dt;
    }

    computeState(t);
    pitch = computePitch(t);
    foil.rotate(-foil.phi-pitch+PI);
    Heave = computeHeave(t);
    foil.translate(0.,Heave - foil.xc.y);
    println("AoA: "+(pitch-AoF)*180/PI);

    flow.update(foil);flow.update2();
    t += dt;
    
    print("t="+nfs(t/resolution,2,2)+";  ");
  }
  
  void display() {
    flood.display(flow.u.curl());
    foil.display();
    //foil.displayVector(foil.pressForce(flow.p));
  }
}
