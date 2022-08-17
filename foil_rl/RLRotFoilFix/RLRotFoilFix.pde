RotatingFoil test;
SaveData dat;

float maxT;
String datapath = "saved/";

int Re = 1173, nflaps = 20;
float hc = 1, chord = 1.0;
//changed parameters
float theta0=0.3, AD = 1, Phi=1, St = 0.3;
String Sp;
float CT = 0, CL = 0, CP = 0;
float y = 0, angle = 0;
int resolution = 32, xLengths=10, yLengths=12, zoom = 3;
int picNum = 10;
float tCurr = 0, tStep = .005;


void settings(){
  size(zoom*xLengths*resolution, zoom*yLengths*resolution);
}
void setup() {

  setUpNewSim();
}
void draw() {
  test.update2();
  test.display();
  
  
  double angle=(test.foil.phi-PI)*180/PI;
  PVector forces = test.foil.pressForce(test.flow.p);
  PVector coordinate = test.foil.xc;
  y = coordinate.y - yLengths * resolution / 2.;
  PVector deltaxy = test.foil.dxc;
  float phivel = test.foil.dphi*resolution/test.dt;
  float vely = deltaxy.y*resolution/test.dt;
  float M = test.foil.pressMoment(test.flow.p);
  CT += -forces.x / resolution * 2;
  CP  = (-(M*phivel)-(forces.y*vely));
  float Eta  = -forces.x/CP*100;
  Sp = SparsePressure(test);
  //dat.addData(test.t, test.foil.pressForce(test.flow.p), test.foil, test.flow.p);
  //dat.addDataSimple(test.t, test.foil.pressForce(test.flow.p));
  dat.output.println(test.t+" " + test.foil.pressForce(test.flow.p).x+ " "+test.foil.pressForce(test.flow.p).y +" "+ angle +" "+ y +" "+ Eta +" "+ Sp +";");
  if(test.t>=maxT/20*18){
    picNum--;
    if(picNum <= 0){
      saveFrame("saved/" + "/" +"frame-#######.png");
      picNum = 10;
    }
  }
  
  if (test.t>=maxT) {
    dat.finish();
  }
}

void setUpNewSim(){
  //float AoA = 5.0*runNum;
  float dAoA = theta0*PI/180., uAoA = dAoA;
  float dA = AD, uA = dA;
  float phi = Phi*PI/180.;
  //if(St < 0.2){
    //maxT = chord/0.3*nflaps;
  //}
  //else{
  maxT = chord/St*nflaps;
  //}
  test = new RotatingFoil(resolution, xLengths, yLengths, tStep, Re, true);
  test.setFlapParams(St, dAoA, uAoA,dA,uA,phi);
  dat = new SaveData(datapath + "/force.txt", test.foil.coords, resolution, xLengths, yLengths, zoom);
  dat.output.println("t"+" "+"fx"+" "+"fy"+" "+"theta"+" "+"y"+" "+"Eta"+" "+"SparsePressure");
}
public String SparsePressure(RotatingFoil test){
  float[] Pressure = new float[0];
  String p;
  for (int i = 0; i < (test.foil.coords.size()) / 20; i++)
  {
    Pressure = append(Pressure, test.flow.p.linear(test.foil.coords.get(i * 20).x, test.foil.coords.get(i * 20).y));
  }
  Pressure = append(Pressure, test.flow.p.linear(test.foil.coords.get(199).x, test.foil.coords.get(199).y));
  p = String.valueOf(Pressure[0]);
  for (int i = 1; i < Pressure.length; i++)
  {
    p = p + "_" + String.valueOf(Pressure[i]);
  }
  return p;
}
