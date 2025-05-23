import processing.core.*; 
import processing.data.*; 
import processing.event.*; 
import processing.opengl.*; 

import java.util.*; 
import org.apache.xmlrpc.*; 
import java.util.concurrent.*; 
import java.util.concurrent.Semaphore; 
import com.alibaba.fastjson.JSONObject; 
import java.util.Iterator; 
import java.util.ListIterator; 

import org.apache.commons.codec.binary.*; 
import org.apache.commons.codec.*; 
import org.apache.commons.codec.digest.*; 
import org.apache.commons.codec.language.bm.*; 
import org.apache.commons.codec.language.*; 
import org.apache.commons.codec.net.*; 
import org.apache.commons.codec.cli.*; 
import org.apache.commons.logging.*; 
import org.apache.commons.logging.impl.*; 
import com.alibaba.fastjson.annotation.*; 
import com.alibaba.fastjson.asm.*; 
import com.alibaba.fastjson.codegen.*; 
import com.alibaba.fastjson.*; 
import com.alibaba.fastjson.parser.*; 
import com.alibaba.fastjson.parser.deserializer.*; 
import com.alibaba.fastjson.serializer.*; 
import com.alibaba.fastjson.support.spring.*; 
import com.alibaba.fastjson.util.*; 
import org.apache.ws.commons.serialize.*; 
import org.apache.ws.commons.util.*; 
import org.apache.xmlrpc.applet.*; 
import org.apache.xmlrpc.secure.sunssl.*; 
import org.apache.xmlrpc.util.*; 
import org.xml.sax.*; 
import uk.co.wilson.xml.*; 
import uk.org.xml.sax.*; 
import org.apache.xmlrpc.*; 
import org.apache.xmlrpc.secure.*; 

import java.util.HashMap; 
import java.util.ArrayList; 
import java.io.File; 
import java.io.BufferedReader; 
import java.io.PrintWriter; 
import java.io.InputStream; 
import java.io.OutputStream; 
import java.io.IOException; 

public class ThreePartFish extends PApplet {

//Added by XYC
//Added here-1





//over-1

//FishRL
Fish3 test;
BDIM flow;
//Bread fish;
FloodPlot flood;
float time=0;
int resolution = 16;
float Re_test = 2000;//Re=6000
float St_test = 0.5f;//Str=0.45

// Added here-2
SaveData dat;
float maxT;
PrintWriter output;
//over-2

//state 是pressforce什么的吗


// void setup(){
//    
//   test = new Fish3(resolution, Re_test, St_test, true);
// }
// void draw(){
//   test.update();
//   test.display();
  
// }

// Added here-3
int picNum = 10;
float tStep = .005f;
String datapath = "saved" + "/";
int EpisodeNum = 1;
int EpisodeTime = 50, ActionTime = 0; //TODO: actionTime = 5
float CT = 0, CL = 0, CP = 0;
// float AoA = 0, A = 160;
float A1 = 0.2f, A2 = 0.3f, A3 = 0;
float ooo = 0;
float Eta = 0;
float angle1 = 0, angle2 = 0;
float nextactionAoA=0,nextactionA=0;
String p;
XmlRpcClient client;
WebServer server;
// float[] ycoords;
int calllearn = 1,Num_steps=3;

//remote info exchange
ArrayList state = new ArrayList();
float[] action = new float[3];  //先令action size=3
float reward_buffer = 0;
Boolean done = false;
int reset_flag = 0;
// Semaphore to provide asynchronization
Semaphore action_sem = new Semaphore(0);
Semaphore state_sem = new Semaphore(0);

public void setup()
{
  size(1600, 800); 
  //surface.setVisible(false);
  println(args);

  try
  {

    server = StartServer(PApplet.parseInt(args[0]));

  }
  catch (Exception ex)
  {
    println(ex);
  }
  setUpNewSim(EpisodeNum);
}

public void draw(){
  if (test.t <= EpisodeTime){
    if(reset_flag == 1){
    //  dat.finish();
    //  output.close();
     EpisodeNum +=1;
    //  A = 0;
    //  AoA = 80;
     A1 = 0.2f;
     A2 = 0.3f;
     A3 = 0;
     setUpNewSim(EpisodeNum);
     test.t = 0;
     done = false;
     reset_flag = 0;

// TODO 这里要加哪些state，以及state的计算方式待修改

     ArrayList state_cn = multy_state(angle1,angle2,0,0,0,0);

     state.clear();
     state.add(state_cn);
    }
    if(done == true){// done == true stop sim
        //continue
        // dat.finish();
        // output.close();

    }
    if (test.t<ActionTime){
      // A = 80;
      // AoA = 0;
      // test.update2(AoA, A);
      A1 = 0.2f;
      A2 = 0.3f;
      A3 = 0;
      test.update(A1, A2, A3);
      test.display();
//TODO foil环境里是通过NACA foil的pressforce计算的力，但是fish3里没有对应的力的计算

//TODO 计算角度，关于是哪个joint的角度我也不知道，所以就直接设为2和3的了
      angle1 = (test.joint2.phi - PI) / PI * 180.f;
      angle2 = (test.joint3.phi - PI) / PI * 180.f;
//TODO 没有相关的计算位置与速度的接口！fish3里看起来只有角速度相关的？

      ArrayList state_cn = multy_state(angle1,angle2,0,0,0,0);
      state.clear();
      state.add(state_cn);
    }
    else if(test.t>=ActionTime){
    // TODO: read action from server logger
    //System.out.println("[debug]before action...");
    float[] NextAction = callActionByRemote();

    A1 = NextAction[0];
    A2 = NextAction[1];
    A3 = NextAction[2];

    test.update(A1, A2, A3);
    test.display();

//TODO 和上面一样的问题，state什么的

    angle1 = (test.joint2.phi - PI) / PI * 180.f;
    angle2 = (test.joint3.phi - PI) / PI * 180.f;

    ArrayList state_cn = multy_state(angle1,angle2,0,0,0,0);
    state.clear();
    state.add(state_cn);

    ooo = test.omega_c(A1, A2, A3);
    reward(ooo);
    ooo = 0;


    // dat.output.println(test.t + " " + angle1 + " " + angle2 + ";");
    //println("EpisodeNUm=" + EpisodeNum);
    if (test.t > 40)
      {
        picNum--;
        if (picNum <= 0)
        {
          saveFrame(datapath + "Episode" + str(EpisodeNum) + "/" + "frame-#######.png");
          picNum = 10;
        }
      }
    }
  }

  else
  {
    done = true;
    // if is out of  timestemp, block here, and not response to the action
    callActionByRemote();
    state_sem.release();
    //dat.finish();
    //output.close();
    //EpisodeNum += 1;
    //setUpNewSim(EpisodeNum);
    //test.t = 0;
  }
}


// remote call action
public float[] callActionByRemote()
{
  try
  {
    // action_sem will wait, utill the server receive the action
    action_sem.acquire();
  }
  catch (Exception ex)
  {
    System.out.println(ex);
  }
  //System.out.println("[debug]return refreshed action. action0:"+ String.valueOf(action[0])+ "action1:"+ String.valueOf(action[1]));
  return action;
}

//TODO 写reward函数
public void reward(float ooo)
{
  // TODO: !!! update reward , done and state in buffer
  reward_buffer = ooo;
  //release state semaphore to let server return the resulted state
  state_sem.release();
}
// start a server
public WebServer StartServer(int port)
{
  println(port);
  WebServer server = new WebServer(port);
  server.addHandler("connect", new serverHandler());
  server.start();

  System.out.println("Started server successfully.");
  System.out.println("Accepting requests. (Halt program to stop.)");
  return server;
}
// server handler to provide api
public
class serverHandler
{

    public String Step(String actionInJson)
      {
        // sparse json
        JSONObject input_object = JSONObject.parseObject(actionInJson);
        JSONObject output_object = new JSONObject();
        //refresh action TODO: need to pre-processing before using the raw data
        action[0] = input_object.getFloat("a1");
        action[1] = input_object.getFloat("a2");
        action[2] = input_object.getFloat("a3");
        // release action, and then
        action_sem.release();
        //println("action:", action[0], "  ", action[1]);

        // query new state, reward , done and wait

        //println(action_sem);
        //println(state_sem);
        try {
                state_sem.acquire();
        } catch (InterruptedException e) {
            // do something, if you wish
            println(e);
            println("[Error] state do not refresh");
        } finally {
          //  state_sem.release();
        }
        output_object.put("state", state);
        output_object.put("reward", reward_buffer);
        output_object.put("done", done);


        return output_object.toJSONString();
      }

    public String query_state()
      {
        JSONObject output_object = new JSONObject();
        output_object.put("state", state);
        return output_object.toJSONString();
      }

    public String reset()
      {
        reset_flag = 1;
        test.t = 0; // Very important, which make the lilypad out of end loop
        // release action, and then
        action_sem.release();
        // query new state, reward , done and wait
        //TODO: maybe some bug
        try {
            state_sem.acquire();
        } catch (InterruptedException e) {
            // do something, if you wish
            print(e);
        } finally {
           // state_sem.release();
        }
        return "success";
      }


}
public ArrayList multy_state(float py,float ptheta,float pfx,float pfy,float peta,float psp) {
  ArrayList list = new ArrayList();
  list.add(py);list.add(ptheta);
  // list.add(pfx);list.add(pfy);
  list.add(0);list.add(0);
  if(Double.isNaN(psp)){
  psp = 0;
  }
  list.add(peta);list.add(psp);
 return list;
}

public void
setUpNewSim(int runNum)
{
  new File(datapath + "Episode" + str(runNum)).mkdir();
  // test = new RotatingFoil(resolution, xLengths, yLengths, tStep, Re, true, true);
  test = new Fish3(resolution, Re_test, St_test, true);
  // dat = new SaveData(datapath + "Episode" + str(runNum) + "/force.txt", resolution);
  // dat.output.println("t" + " " + "fx" + " " + "fy" + " " + "theta" + " " + "y");
  // output = createWriter(datapath + "Episode" + str(runNum) + "/output.csv");
  // output.println("" + "Action1" + "," + "Action2");
  println("Episode" + str(runNum) + " start!");

  A1 = 0.2f;
  A2 = 0.3f;
  A3 = 0;
}
//over-3
/*********************************************************
Simple model of a swimming plesiosaur
 
 The foils both use simple harmonic pitch and heave 
 flapping. You can set the horizontal spacing and phase 
 difference between the foils. 
 
 Note that the example code needs increased domain and resolution 
 should use QUICK convection to obtain highly accurate results. 

Example code:

AncientSwimmer plesiosaur;
BDIM flow;
FloodPlot plot;
PrintWriter output;
int n=(int)pow(2,8);      // number of grid points
float L = n/8., St = 0.4; // chord length, Strouhal number
float t = 0;              // time
 
void setup(){
  size(1000,500);                                     // display window size
  Window view = new Window(n,n/2);                    // mapping from grid to display
  plesiosaur = new AncientSwimmer(1.25*L,n/4,L,3*L,   // define the geometry...
                                  1.75*PI,St,view);   //    and motion
  flow = new BDIM(n,n/2,1.0,plesiosaur);              // define fluid
  plot = new FloodPlot( view );                       // define plot...
  plot.setLegend("Vorticity",-0.5,0.5);               //    and legend
  output = createWriter("plesiosaur/out.csv");        // open output file
}

void draw(){
  t += flow.dt;                                       // update the time
  plesiosaur.update(t,flow.dt);                       // update the geometry
  flow.update(plesiosaur); flow.update2();            // 2-step fluid update
  plot.display(flow.u.curl());                        // display the vorticity
  plesiosaur.display();                               // display the geometry
  
  PVector[] forces = plesiosaur.pressForces(flow.p);  // pressure force on both bodies
  float ts = St*t/(2.*L);                             // time coefficient
  float front = 2.*forces[0].x/L;                     // thrust coefficient on front
  float back = 2.*forces[1].x/L;                      // thrust coefficient on back
  output.println(""+ts+","+front+","+back);           // print to file

  if(ts>=4){                                          // finish after 4 cycles
    output.close();
    exit();
  }
}
*******************************************************/

class AncientSwimmer extends BodyUnion{
  float x0,y0,L,s,lead,St,pamp;
  
  AncientSwimmer( float x0, float y0, float L, float s, float lead, float St, Window view){
//  make geometry
    super(new NACA(x0,y0,L,0.16f,view),        // front foil
          new NACA(x0+s,y0,L,0.16f,view));     // back foil

// save parameters
    this.L = L;                              // Foil coord
    this.x0 = x0; this.y0 = y0;              // Starting position of front foil
    this.s = s; this.lead = lead;            // Spacing and phase lag
    this.St = St;                            // Strouhal number of motion

// set pitch amplitude to get 10 degree AOA
    pamp = atan(PI*St)-PI/18.f;               
    
// set initial state
    bodyList.get(0).follow(kinematics(0,0,0),new PVector());
    bodyList.get(1).follow(kinematics(s,lead,0),new PVector());

// set color
    bodyColor=color(255);
  }

// update the position of the two foils
  public void update(float t, float dt){
    bodyList.get(0).follow(kinematics(0,0,t),dkinematics(0,t,dt));
    bodyList.get(1).follow(kinematics(s,lead,t),dkinematics(lead,t,dt));
  }
  
// define the foil motion
  public PVector kinematics(float s, float lead, float t){    
    float phase = PI*St*t/L+lead;          // phase
    return new PVector(x0+s,               // x position
                       y0-L*sin(phase),    // y position
                       pamp*cos(phase));   // pitch position
  }
  public PVector dkinematics(float lead, float t, float dt){
    float phase = PI*St*t/L+lead;          // phase
    return new PVector(0,                  // dx
                       -cos(phase)*PI*St*dt,   // dy
                       -pamp*sin(phase)*PI*St/L*dt); // dphi
  }
  
// get pressure force of both foils
  public PVector[] pressForces(Field p){
    PVector f0 = bodyList.get(0).pressForce(p);
    PVector f1 = bodyList.get(1).pressForce(p);
    return new PVector[]{f0,f1};
  }
}
/*********************************************************
solve the BDIM equation for velocity and pressure

  u = del*F+[1-del]*u_b+del_1*ddn(F-u_b)
  
  where 
    del is the zeroth moment of the smoothing kernel integral
    del_1 is the first moment (zero if mu1=false)
    u_b is the body velocity
    F is the fluid equation of motion:
    if(QUICK):
      F(u) = u(t)+\int_t^{t+dt} grad(u*u)+\mu*laplace(u)+g-grad(p)/rho \d t
    else(SEMI-LAGRANGIAN)
      F(u) = u(t,x(t))+\int_t^{t+dt} g-grad(p)/rho \d t
      
    where x(t) is the back-casted location of the grid points
      x(t) = x-\int_t^{t+dt} u \d t

Example code:

BDIM flow;
void setup(){
  size(400,400); 
  int n=(int)pow(2,7);
  flow = new BDIM(n,n,0.,new CircleBody(n/3,n/2,n/8,new Window(n,n)),n/8000.,true);
}
void draw(){
  flow.update();        // project
  flow.update2();       // correct
  flow.p.display(-1,1); // display pressure
}
*********************************************************/
class BDIM{
  int n,m; // number of cells in uniform grid
  float t=0, dt, nu, eps=2.0f;
  PVector g= new PVector(0,0);
  VectorField u,del,del1,c,u0,ub,wnx,wny,distance,rhoi;
  Field p;
  boolean QUICK, mu1=true, adaptive=false;

  BDIM( int n_, int m_, float dt_, Body body, VectorField uinit, float nu_, boolean QUICK_ ){
    n = n_+2; m = m_+2;
    dt = dt_;
    nu=nu_;
    QUICK=QUICK_;
    if(!QUICK && nu!=0) {
      println("Semi-lagrangian advection cannot include explicit value for `nu`.");
      exit();
    }

    u = uinit;
    if(u.x.bval!=0) u.x.gradientExit = true;
    u0 = new VectorField(n,m,0,0);
    p = new Field(n,m);
    if(dt==0) setDt(); // adaptive time stepping for O(2) QUICK

    ub  = new VectorField(n,m,0,0);
    distance =  new VectorField(n, m, 10, 10);    
    del = new VectorField(n,m,1,1);
    del1 = new VectorField(n,m,0,0);
    rhoi = new VectorField(del);
    c = new VectorField(del);
    wnx = new VectorField(n,m,0,0);
    wny = new VectorField(n,m,0,0);
    get_coeffs(body);
  }
  
  BDIM( int n, int m, float dt, Body body, float nu, boolean QUICK, float u_inf){
    this(n,m,dt,body,new VectorField(n+2,m+2,u_inf,0),nu,QUICK);}
  BDIM( int n, int m, float dt, Body body, float nu, boolean QUICK ){
    this(n,m,dt,body,new VectorField(n+2,m+2,1,0),nu,QUICK);}
  
  // If no body is supplied, create a body outside the domain
  BDIM( int n, int m, float dt, VectorField uinit, float nu, boolean QUICK ){
    this(n,m,dt,new CircleBody(-n/2,-m/2,n/10,new Window(0,0,n,m)),uinit,nu,QUICK);}
  
  BDIM( int n, int m, float dt, Body body){this(n,m,dt,body,new VectorField(n+2,m+2,1,0),0,false);}
  
  public void update(){
    // O(dt,dx^2) BDIM projection step:
    c.eq(del.times(rhoi.times(dt))); 
    u0.eq(u);
    VectorField F = new VectorField(u);
    if(QUICK) F.AdvDif( u0, dt, nu );
    else F.advect( dt, u0 );
    updateUP( F, c );
  }
  
  public void update2(){
    // O(dt^2,dt^2) BDIM correction step:
    VectorField us = new VectorField(u), F = new VectorField(u);
    if(QUICK){
      F.AdvDif( u0, dt, nu );
      updateUP( F, c );
      u.plusEq(us); 
      u.timesEq(0.5f);
      if(adaptive) dt = checkCFL();
    }
    else{
      F.eq(u0); 
      F.advect(dt,us,u0);
      VectorField dp = p.gradient().times(rhoi.times(0.5f*dt));
      dp.advect(dt,us,u0);
      updateUP( F.minus(dp), c.times(0.5f), F.minus(ub) );
    }
    t += dt;
  }
  
  public void updateUP( VectorField R, VectorField coeff, VectorField du ){
/*  Seperate out the pressure from the forcing
      del*F = del*R+coeff*gradient(p)
    Approximate update (dropping ddn(grad(p))) which doesn't affect the accuracy of the velocity
      u = del*R+coeff*gradient(p)+[1-del]*u_b+del_1*ddn(R-u_b)
    Take the divergence
      div(u) = div(coeff*gradient(p)+stuff) = 0
    u.project solves this equation for p and then projects onto u
*/
    R.plusEq(PVector.mult(g,dt));
    u.eq(del.times(R).minus(ub.times(del.plus(-1))));
    if(mu1) u.plusEq(del1.times(du.normalGrad(wnx,wny)));
    u.setBC();
    p = u.project(coeff,p);    
  }
  public void updateUP( VectorField R, VectorField coeff){updateUP( R, coeff, R.minus(ub));}

  public void update( Body body ){
    if(body.unsteady()){get_coeffs(body);}else{ub.eq(0.f);}
    update();
  }
  public void update2( Body body ){update2();} // don't need to get coeffs again

  public void get_coeffs( Body body ){
    get_dist(body);
    get_del();
    get_del1();
    get_ub(body);
    get_wn(body);
  }
  
  public void get_ub( Body body ){
    /* Immersed Velocity Field
          ub(x) = U(x)*(1-del(x))
    where U is the velocity of the body */
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
        ub.x.a[i][j] = body.velocity(1,dt,(float)(i-0.5f),j);
        ub.y.a[i][j] = body.velocity(2,dt,i,(float)(j-0.5f));
        //println(" get_ub intro");
    }}
  }
  
  public void get_wn(Body body){
   /* wall normal direction of the closest body point */
   PVector wn;
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      wn = body.WallNormal((float)(i-0.5f),j);
      wnx.x.a[i][j]=wn.x;
      wny.x.a[i][j]=wn.y;
      wn = body.WallNormal(i,(float)(j-0.5f));
      wnx.y.a[i][j]=wn.x;
      wny.y.a[i][j]=wn.y;
    }}   
  }
  
  public void get_dist( Body body ) {
    for ( int i=1 ; i<n-1 ; i++ ) {
      for ( int j=1 ; j<m-1 ; j++ ) {
        distance.x.a[i][j] = body.distance((float)(i-0.5f), j);
        distance.y.a[i][j] = body.distance(i, (float)(j-0.5f));
      }
    }
  }
  
  public void get_del( ){
    /* BDIM zeroth order moment
          del(x) = delta0(d(x))
    where d is the distance to the interface from x */
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
        del.x.a[i][j] = delta0(distance.x.a[i][j]);
        del.y.a[i][j] = delta0(distance.y.a[i][j]);
    }}
    del.setBC();
  }
  
  public void get_del1(){
    /* BDIM first order moment
          del(x) = delta1(d(x))
    where d is the distance to the interface from x */
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
        del1.x.a[i][j] = delta1(distance.x.a[i][j]);
        del1.y.a[i][j] = delta1(distance.y.a[i][j]);
    }}
    del1.setBC();
  }

  
  public float delta0( float d ){
    if( d <= -eps ){
      return 0;
    } else if( d >= eps ){
      return 1;
    } else{
      return 0.5f*(1.f+d/eps+sin(PI*d/eps)/PI);
    } 
  }
  
  public float delta1( float d ){
    if( abs(d) >= eps){
      return 0;
    } else{
        return 0.25f*(eps-sq(d)/eps)-1/TWO_PI*(d*sin(d*PI/eps)+eps/PI*(1+cos(d*PI/eps)));
    } 
  }
  
  public float checkCFL() { 
    return min(u.CFL(nu), 1);
  }

  public void setDt(){
    dt = checkCFL();
    if(QUICK) adaptive = true;
  }

  public void write( String name ){
    PrintWriter output;
    output = createWriter(name);
    output.println(t);
    output.println(dt);
    for ( int i=0; i<n; i++ ){
    for ( int j=0; j<m; j++ ){
      output.println(""+u.x.a[i][j]+", "+u.y.a[i][j]+", "+p.a[i][j]);
    }}
    output.flush(); 
    output.close();
  }

  public void resume( String name ){
    float[] data;
    String[] stuff = loadStrings(name);
    t = PApplet.parseFloat(stuff[0]);
    dt = PApplet.parseFloat(stuff[1]);
    for ( int i=0; i<n; i++ ){
    for ( int j=0; j<m; j++ ){
      data = PApplet.parseFloat(split(stuff[2+i*m+j],','));
      u.x.a[i][j] = data[0];
      u.y.a[i][j] = data[1];
      p.a[i][j] = data[2];
    }}
  }
}
/*************************
Audrey's Blind Cave Fish Test Class
  Example test case in which a cylinder passes by a foil

Example code:

SaveData dat;
BlindFish test;
int resolution = 64, xLengths=5, yLengths=3, zoom = 2;    // choose the number of grid points per chord, the size of the domain in chord units and the zoom of the display

void settings(){
    size(zoom*xLengths*resolution, zoom*yLengths*resolution);
}
void setup(){
  float xStart = 1, yDist = 0.06;        // choose the initial horizontal position of the cylinder and vertical separation between the foil and the cylinder
  test = new BlindFish(resolution, xLengths, yLengths, xStart , yDist, zoom);
  dat = new SaveData("saved/pressure.txt",test.body.bodyList.get(0).coords,resolution,xLengths,yLengths,zoom);    // initialize the output data file with header information
}
void draw(){
  if(test.t<2){  // run simulation until t<Time
    test.update();
    test.display();
    dat.addData(test.t, test.flow.p);    // add the pressure arounf the foil to the data file
    saveFrame("saved/frame-####.png");   // save images. To make a movie use Tools > Movie Maker
  }else{  // close and save everything when t>Time
    dat.finish();
    exit();
  }
}

void keyPressed(){   // close and save everything when the space bar is pressed
    dat.finish();
    exit();
}
***********************/

class BlindFish{
  final int n,m, resolution, NT=2;  //nXm domain with resolution grid points per chord. displays and saves every NT computational time step
  float dt = 0, t, t0, Re=50000, cDiameter = 0.5f; //choose time step if using semi-lagrangian, Reynolds number if using QUICK, and cylinder diameter in chord units
  boolean QUICK = true, order2 = true; // choose whether to use QUICK and second order time integration
  BodyUnion body; BDIM flow; FloodPlot flood, flood2; Window window, window2;
  
  BlindFish( int resolution, int xLengths, int yLengths, float xStart , float yDist, float zoom){
    n = xLengths*resolution;
    m = yLengths*resolution;
    this.resolution = resolution;
    float xFoil = 1.0f/3.0f;     // position of the foil in % of the domain size
    float yFoil = 1.0f/2.0f;
    float yStart = yLengths*yFoil-yDist-0.06f-0.5f*cDiameter;   // position of the cylinder at the beginning of the simulation
    
    t0 = xStart-xFoil*xLengths;    // choose initial time such that the cylinder is passing the foil at time=0
    t=t0;

    smooth();
//    window = new Window(n,m);      // display the entire domain
    window = new Window(n/6,m/5,n/2,m/2);
    window2 = new Window(n/6,m/5,n/2,m/2);    // zoom on the foil

    body = new BodyUnion( new NACA(xFoil*n,yFoil*m,resolution,0.12f,window), new CircleBody(xStart*resolution,yStart*resolution,cDiameter*resolution,window));   // create foil and cylinder

    flow = new BDIM(n,m,dt,body,(float) resolution/Re,QUICK);

    flood = new FloodPlot(window);
    flood.range = new Scale(-0.75f,0.75f);
    flood.setLegend("vorticity");

    flood2 = new FloodPlot(window2);
    flood2.range = new Scale(-0.5f,0.5f);
    flood2.setLegend("pressure");
  }
  
  public void update(){
    for ( int i=0 ; i<NT ; i++ ) {
    if (QUICK){dt = flow.dt;}
     body.bodyList.get(1).translate(dt,0);    // translate the cylinder of dt*velocity. here the cylinder moves one chord length per unit time
     flow.update(body);
     if (order2) {flow.update2(body);}
    print("t="+nfs(t,2,2)+";  ");
    t += dt/resolution;
    }
  }
  
  public void display(){
//    flood.display(flow.u.curl());
//    body.display();
//    flood.displayTime(t);
    flood2.display(flow.p);
    body.display(window2);
    flood2.displayTime(t);
  }
}
/********************************
 Body class
 
 this is the parent of all the body classes
 
 it defines the position and motion of a convex body. 
 
 points added to the body shape must be added counter
 clockwise so that the convexity can be tested.
 
 the easiest way to include dynamics is to use follow()
 which follows the path variable (or mouse) or to use
 react() which integrates the rigid body equations.
 You can also translate() and rotate() directly.
 
Example code:
 
Body body;
void setup(){
  size(400,400);
  body = new Body(0,0,new Window(0,0,4,4));
  body.add(0.5,2.5);
  body.add(2.75,0.25);
  body.add(0,0);
  body.end();
  body.display();
  println("The distance "+body.distance(3.,1.)+" should equal "+sqrt(.5));
}
void draw(){
  background(0);
  body.follow();
  body.display();
}
void mousePressed(){body.mousePressed();}
void mouseReleased(){body.mouseReleased();}
void mouseWheel(MouseEvent event){body.mouseWheel(event);}

********************************/

class Body {
  Window window;
  int bodyColor = 0xff993333;
  final int bodyOutline = 0xff000000;
  final int vectorColor = 0xff000000;
  PFont font = loadFont("Dialog.bold-14.vlw");
  float phi=0, dphi=0, dotphi=0, ddotphi=0;
  float mass=1, I0=1, area=0;
  ArrayList<PVector> coords=new ArrayList<PVector>();
  /////axis initial
  boolean ifaxis = false;
  //ArrayList<Axis> axis=new ArrayList<Axis>();//new
  /////
  int n;
  boolean convex=false;
  boolean pressed=false, xfree=true, yfree=true, pfree=true;
  PVector xc, dxc=new PVector(), dotxc=new PVector(), ddotxc=new PVector();
  PVector handle=new PVector(), ma=new PVector();
  OrthoNormal orth[];
  Body box;

  Body( float x, float y, Window window ) {
    this.window = window;
    xc = new PVector(x, y);
  }

  public void add( float x, float y ) {
    coords.add( new PVector( x, y ) );
  }
  

  public void end(Boolean closed) {
    n = coords.size();
    orth = new OrthoNormal[closed?n:n-1];
    getOrth(); // get orthogonal projection of line segments
    if(closed) getArea();

    // make the bounding box
    if (n>4) {
      PVector mn = xc.copy(), mx = xc.copy();
      for ( PVector x: coords ) {
        mn.x = min(mn.x, x.x);
        mn.y = min(mn.y, x.y);
        mx.x = max(mx.x, x.x);
        mx.y = max(mx.y, x.y);
      }
      box = new Body(xc.x, xc.y, window);
      box.add(mn.x, mn.y);
      box.add(mn.x, mx.y);
      box.add(mx.x, mx.y);
      box.add(mx.x, mn.y);
      box.end();
    }
    
    // check for convexity
    convex = true;
    double_loop: for ( OrthoNormal oi : orth ) {
      for ( OrthoNormal oj : orth ){
        if(oi.distance(oj.cen.x,oj.cen.y)>0.001f) {
          convex = false; 
          break double_loop;
    }}}
  }
  public void end() {
    end(true);
  }

  public void getOrth() {    // get orthogonal projection to speed-up distance()
    for ( int i = 0; i<orth.length ; i++ ) {
      PVector x1 = coords.get(i);
      PVector x2 = coords.get((i+1)%n);
      orth[i] = new OrthoNormal(x1, x2);
    }
  }
  public void getArea() {    // get the polygon area and moment
    float s=0, t=0;
    for ( int i = 0; i<n ; i++ ) {
      PVector p1 = coords.get(i);
      PVector p2 = coords.get((i+1)%n);
      float x1 = p1.x-xc.x, x2 = p2.x-xc.x, y1 = p1.y-xc.y, y2 = p2.y-xc.y, da = x1*y2-x2*y1;
      s -= da;
      t -= (sq(x1)+x1*x2+sq(x2)+sq(y1)+y1*y2+sq(y2))*da;
    }
    area = 0.5f*s;
    I0 = t/12.f;
    mass = area; // default unit density
  }
 
  public void setColor(int c) {
    bodyColor = c;
  }
  public void display() {
    display(bodyColor, window);
  }
  public void display(Window window) {
    display(bodyColor, window);
  }
  public void display( int C) { 
    display(C, window);
  }
  public void display( int C, Window window ) { // note: can display while adding
    //    if(n>4) box.display(#FFCC00);
    fill(C);
    //noStroke();
    stroke(bodyOutline);
    strokeWeight(1);
    beginShape();
    for ( PVector x: coords ) vertex(window.px(x.x), window.py(x.y));
    endShape(CLOSE);
}
  public void displayVector(PVector V) {
    displayVector(vectorColor, window, V, "Force", true);
  }
  public void displayVector(int C, Window window, PVector V, String title, boolean legendOn) { // note: can display while adding
    //    if(n>4) box.display(#FFCC00);
    float Vscale=10;
    float circradius=6; //pix
    
    stroke(C);
    strokeWeight(2);
    line(window.px(xc.x), window.py(xc.y), window.px(xc.x-Vscale*V.x), window.py(xc.y-Vscale*V.y));
    
    fill(C); 
    noStroke();
    ellipse(window.px(xc.x-Vscale*V.x), window.py(xc.y-Vscale*V.y), circradius, circradius);
    
    if (legendOn){
        textFont(font);
        fill(C);
        float spacing=20;
        int x0 = window.x0, y1 = window.y0+window.dy;
        textAlign(LEFT,BASELINE);
        String ax = ""+V.x;
        String ay = ""+V.y;
        text(title + " X: " + ax.substring(0,min(ax.length(),5)),x0+spacing,y1-2*spacing);
        text(title + " Y: " + ay.substring(0,min(ay.length(),5)),x0+spacing,y1-spacing);
    }
  }

  public float distance( float x, float y ) { // in cells
    float dis;
    if (n>4) { // distance to bounding box
      dis = box.distance(x, y);
      if (dis>3) return dis;
    }
    
    if(convex){ // distance to convex body
      // check distance to each line, choose max
      dis = -1e10f;
      for ( OrthoNormal o : orth ) dis = max(dis, o.distance(x, y));
      return dis;
    } else {   // distance to non-convex body
      // check distance to each line segment, choose min
      dis = 1e10f;
      for( OrthoNormal o: orth ) dis = min(dis,o.distance(x,y,false));
      return (wn(x,y)==0)?dis:-dis; // use winding to set inside/outside
    }
  }
  public int distance( int px, int py) {     // in pixels
    float x = window.ix(px);
    float y = window.iy(py);
    return window.pdx(distance( x, y ));
  }

  public int wn( float x, float y){
    // Winding number. If wn==0 the point is inside the body
    int wn=0;
    for ( int i = 0; i<coords.size()-1; i++ ){      
      float yi = coords.get(i).y;    // y value of point i
      float yi1 = coords.get(i+1).y; // y value of point i+1
      OrthoNormal o = orth[i];       // segment from i to i+1
      if(yi <= y){                   // check for positive crossing
        if(yi1 > y && o.distance(x,y)>0) wn++;
      }else{                         // check for negative crossing
        if(yi1 <= y && o.distance(x,y)<0) wn--;
      }
    }
    return wn;
  }

  public PVector WallNormal(float x, float y  ) {
    PVector wnormal = new PVector(0, 0);
    float dis = -1e10f;
    float dis2 = -1e10f;
    if (n>4) { // check distance to bounding box
      if ( box.distance(x, y)>3) return wnormal;
    }
    // check distance to each line, choose max
    for ( OrthoNormal o : orth ) {
      dis2=o.distance(x, y);
      if (dis2>dis) {
        dis=dis2;
        wnormal.x=o.nx;
        wnormal.y=o.ny;
      }
    }
    return wnormal;
  }

  public float velocity( int d, float dt, float x, float y ) {
    // add the rotational velocity to the translational
    PVector r = new PVector(x, y);
    r.sub(xc);
    if (d==1) return (dxc.x-r.y*dphi)/dt;
    else     return (dxc.y+r.x*dphi)/dt;
  }

  public PVector velocity_new(float dt, float x, float y){
    //newly modified by TreeSun
    PVector r = new PVector(x, y);
    PVector velo = new PVector(x, y);
    r.sub(xc);
    velo.x= (dxc.x-r.y*dphi)/dt;//u
    velo.y= (dxc.y+r.x*dphi)/dt;//v
    return velo;
  }

  public boolean unsteady(){return (dxc.mag()!=0)|(dphi!=0);}
  public boolean unsteady(boolean flag){return flag;}

  public void translate( float dx, float dy ) {
    dxc = new PVector(dx, dy);
    xc.add(dxc);
    for ( PVector x: coords ) x.add(dxc);
    for ( OrthoNormal o: orth   ) o.translate(dx, dy);
    if (n>4) box.translate(dx, dy);
  }

  public void rotate( float dphi ) {
    this.dphi = dphi;
    phi = phi+dphi;
    float sa = sin(dphi), ca = cos(dphi);
    for ( PVector x: coords ) rotate( x, xc, sa, ca ); 
    getOrth(); // get new orthogonal projection
    if (n>4) box.rotate(dphi);
  }
  public void rotate( PVector x, PVector xc, float sa, float ca ) {
    PVector z = PVector.sub(x, xc);
    x.x = ca*z.x-sa*z.y+xc.x;
    x.y = sa*z.x+ca*z.y+xc.y;
  }
  public void rotate( PVector x, float sa, float ca ) {rotate(x,new PVector(),sa,ca);}
  public void rotate_init(float phi){
    float sa = sin(phi), ca = cos(phi);
    for ( PVector x: coords ) rotate( x, xc, sa, ca ); 
    getOrth(); 
    if (n>4) box.rotate(phi);
  }
  
  // Move body to path=(x,y,phi)
  public void follow(PVector path) {
    PVector d = path.copy().sub(xc.copy().add(handle)); // distance to point;
    d.z = (d.z-phi);                                    // arc length to angle
    translate(d.x,d.y); rotate(d.z);                    // translate & rotate
  }
  public void follow() {
    if(pressed) follow(new PVector(window.ix(mouseX),window.iy(mouseY)));
  }
  public void follow(PVector path, PVector dpath){
    follow(path);
    dxc = new PVector(dpath.x, dpath.y);
    dphi = dpath.z;
  }
  
  public void mousePressed() {
    if (distance( mouseX, mouseY )<1) {
      pressed = true;
      handle = new PVector(window.ix(mouseX),window.iy(mouseY)).sub(xc);
    }
  }
  public void mouseReleased() {
    pressed = false;
    dxc = new PVector(); dphi = 0; // Don't include velocity
  }
  public void mouseWheel(MouseEvent event) {
    if (distance( mouseX, mouseY )<1) rotate(event.getCount()/PI/100);
  }

  public PVector pressForce ( Field p ) {
    PVector pv = new PVector(0, 0);
    for ( OrthoNormal o: orth ) {
      float pdl = p.linear( o.cen.x, o.cen.y )*o.l;
      pv.add(pdl*o.nx, pdl*o.ny, 0);
    }
    return pv;
  }
  
  public float pressMoment ( Field p ) {
    float mom = 0;
    for ( OrthoNormal o: orth ) {
      float pdl = p.linear( o.cen.x, o.cen.y )*o.l;
      mom += pdl*(o.ny*(o.cen.x-xc.x)-o.nx*(o.cen.y-xc.y));
    }
    return mom;
  }

  public float pressPower_old ( Field p, float dt ) {//official
    float power = 0;
    for ( OrthoNormal o: orth ) {
      float x = o.cen.x, y = o.cen.y,
            u = velocity( 1, dt, x, y ),
            v = velocity( 2, dt, x, y ),
            pdl = p.linear( x, y )*o.l;
      power += pdl*(u*o.nx+v*o.ny);
    }
    return power;
  }

  public float pressPower ( Field p, float dt ) {//newly modified by TreeSun
    float power = 0;
    for ( OrthoNormal o: orth ) {
      PVector velo=velocity_new( dt, o.cen.x, o.cen.y );
      float x = o.cen.x, y = o.cen.y,
            u = velo.x,
            v = velo.y,
            pdl = p.linear( x, y )*o.l;
      power += pdl*(u*o.nx+v*o.ny);
    }
    return power;
  }
  // compute body reaction to applied force and moment
  public void react (PVector force, float moment, float dt) {
    /* X,Y */
    if(xfree|yfree){
      // rotate to body-fixed axis
      float sa = sin(phi), ca = cos(phi);
      rotate(force,-sa,ca);
      rotate(ddotxc,-sa,ca);
      // compute acceleration (in global axis)
      ddotxc.x = (force.x+ma.x*ddotxc.x)/(mass+ma.x);
      ddotxc.y = (force.y+ma.y*ddotxc.y)/(mass+ma.y);
      rotate(ddotxc,sa,ca);
      // compute velocity and delta
      dotxc.add(ddotxc.copy().mult(dt));
      dxc = dotxc.copy().mult(dt);
      // translate
      if(!xfree) {dxc.x=0; dotxc.x=0; ddotxc.x=0;}
      if(!yfree) {dxc.y=0; dotxc.y=0; ddotxc.y=0;}
      translate(dxc.x+0.5f*ddotxc.x*sq(dt),dxc.y+0.5f*ddotxc.y*sq(dt));
    } else{
      dxc=new PVector(); dotxc=new PVector(); ddotxc=new PVector();
    }
    /* Phi */
    if(pfree){
      // compute acceleration & velocity
      ddotphi = (moment+ma.z*ddotphi)/(I0+ma.z);
      dotphi += ddotphi*dt;
      // compute delta and position
      dphi = dotphi*dt;
      rotate(dphi+0.5f*ddotphi*sq(dt));
    } else{
      dphi=0; dotphi=0; ddotphi=0;
    }
  }
  public void react (BDIM flow) {
    PVector f = pressForce(flow.p).mult(-1);
    float m = pressMoment(flow.p)*(-1);
    react(f,m,flow.dt);
  }

  // check if motion is within box limits
  public boolean check_phi_free(float m, float phi_high, float phi_low){
    return !(phi<phi_low & (m<0 | dotphi<0)) & 
           !(phi>phi_high & (m>0 | dotphi>0));
  }
  public boolean check_y_free(float fy, float y_high, float y_low){
    return !(xc.y<y_low & (fy<0 | dotxc.y<0)) & 
           !(xc.y>y_high & (fy>0 | dotxc.y>0));
  }

}
/********************************
 EllipseBody class
 
 simple elliptical body extension
 ********************************/
class EllipseBody extends Body {
  float h, a; // height and aspect ratio of ellipse
  int m = 40;

  EllipseBody( float x, float y, float _h, float _a, Window window) {
    super(x, y, window);
    h = _h; 
    a = 1.f/_a;
    float dx = 0.5f*h*a, dy = 0.5f*h;
    for ( int i=0; i<m; i++ ) {
      float theta = -TWO_PI*i/((float)m);
      add(xc.x+dx*cos(theta), xc.y+dy*sin(theta));
    }
    end(); // finalize shape

    ma = new PVector(PI*sq(dy),PI*sq(dx),0.125f*PI*sq(sq(dx)-sq(dy)));
  }
}
/* CircleBody
 simplified rotation and distance function */
class CircleBody extends EllipseBody {

  CircleBody( float x, float y, float d, Window window ) {
    super(x, y, d, 1.0f, window);
  }

  public float distance( float x, float y) {
    return mag(x-xc.x, y-xc.y)-0.5f*h;
  }

  public void rotate(float _dphi) {
    dphi = _dphi;
    phi = phi+dphi;
  }
  
}
/********************************
BodyUnion class

This class combines an array of body instances by a union operator
which `adds` bodies to the flow. If two bodies are given, they are added
automatically. Bodies can also be 'add'ed individually. 

Other operations are possible but haven't been coded up yet.

SaveArray is a writer class that outputs all the values for the array 
in one line. At this point, only the pressure forces have been coded.

Example code:

BodyUnion body;
void setup(){
  size(400,400);
  Window view = new Window(100,100);
  body = new BodyUnion( new NACA(30,30,20,0.2,view),
                        new CircleBody(70,30,15,view));
  body.add(new CircleBody(30,70,15,view));
}
void draw(){
  background(0);
  body.follow(); // uncomment to move as a group
  //for (Body child : body.bodyList) child.follow(); // uncomment to move individually
  body.display();
}
void mousePressed(){body.mousePressed();}
void mouseReleased(){body.mouseReleased();}
********************************/
class BodyUnion extends Body{
  ArrayList<Body> bodyList = new ArrayList<Body>();  //This is a container for all bodies

  BodyUnion(float x, float y, Window window){ 
    super(x, y, window);
  }

  BodyUnion(Body a, Body b){
    super(a.xc.x,a.xc.y,a.window);
    add(a); add(b);
  }
  BodyUnion(Body a, Body b, Body c){//for fish3 3.24.2022
    super(a.xc.x,a.xc.y,a.window);
    add(a); add(b); add(c);
  }

  public void add(Body body){
    bodyList.add(body);
    area += body.area;
  }
  
  public void recenter(){
    xc = new PVector(); 
    for (Body b : bodyList){xc.add(b.xc.copy().mult(b.area));}
    xc = xc.div(area);
  }
  public void recenter(float x, float y){xc = new PVector(x,y);}
  
  public void display(int C, Window window ){
    for ( Body body : bodyList ){
        body.display(C, window);
    }
  }
  
  public float distance(float x, float y){
    float d = 1e6f;
    for ( Body body : bodyList )
      d = min(d,body.distance(x,y));
    return d;
  }  
  
  public PVector WallNormal(float x, float y){
    float w[] = get_weights(x,y);
    PVector m = new PVector(0,0);
    for ( int i = 0; i<bodyList.size(); i++ ) {
      PVector n = bodyList.get(i).WallNormal(x,y);
      m.add(n.mult(w[i]));
    }
    return m;
  }
  
  public float velocity( int d, float dt, float x, float y ){
    float w[] = get_weights(x,y);
    float v = 0;
    for ( int i = 0; i<bodyList.size(); i++ ) {
      float u = bodyList.get(i).velocity(d,dt,x,y);
      v += u*w[i];
    }
    return v;
  }

  public void translate(float dx, float dy){
    xc.add(new PVector(dx, dy));
    for (Body body : bodyList) body.translate(dx,dy);
  }
  public void rotate(float dphi){ // rotate around _union_ xc
    float sa = sin(dphi), ca = cos(dphi)-1;
    phi = phi+dphi;
    for (Body body : bodyList){
      PVector z = PVector.sub(body.xc,xc);
      float dx = ca*z.x-sa*z.y;
      float dy = sa*z.x+ca*z.y;
      body.translate(dx,dy);
      body.rotate(dphi);
    }
  }
  //New func
  public void rotate_with_PhaseDif(float thetaM, float omega,float time,Body a,Body b){
    float dphi = omega*thetaM*cos(omega*time);
    float sa = sin(dphi), ca = cos(dphi)-1;
    //PVector z1 = PVector.sub(a.xc,xc);
    //float dx1 = ca*z1.x-sa*z1.y;
    //float dy1 = sa*z1.x+ca*z1.y;
    //body.translate(dx,dy);
    a.rotate(dphi);
    b.rotate(dphi);
  }
  public void initPath(PVector p){
    follow(p);
    for (Body body : bodyList) {
      body.dxc = new PVector(); body.dphi = 0;
    }
  }

  public boolean unsteady(){ 
    boolean unsteady = false;
    for (Body body : bodyList){
      unsteady = unsteady | body.unsteady();
    }
    return unsteady;
  }

  public void react(BDIM flow){
    for (Body body : bodyList) body.react(flow);
  }
  public void react(PVector f, float m, float dt){
    for (Body body : bodyList) body.react(f,m,dt);
  }

  public void mousePressed(){
    super.mousePressed();
    for (Body body : bodyList){body.mousePressed();}
  }  
  public void mouseReleased(){
    super.mouseReleased();
    for (Body body : bodyList){body.mouseReleased();}
  }
  public void mouseWheel(MouseEvent event){
    super.mouseWheel(event);
    for (Body body : bodyList){body.mouseWheel(event);}
  }

// weight the body influences at point x,y
  public float[] get_weights(float x, float y){
    int n = bodyList.size();
    float s=0, weights[]=new float[n];
    for ( int i = 0; i<n; i++ ){
      float d = bodyList.get(i).distance(x,y);
      weights[i] = delta0(-d/3.f); // kernel weight
      s += weights[i];
    }
    for ( int i = 0; i<n & s>0; i++) 
        weights[i] /= s;          // normalize weights
    return weights;
  }
  
  public float delta0( float d ){
    if( d <= -1 ){
      return 0;
    } else if( d >= 1 ){
      return 1;
    } else{
      return 0.5f*(1.f+d+sin(PI*d)/PI);
    } 
  }
}

// Save values on an array of bodies

class SaveArray{
  PrintWriter output;
  
  SaveArray(String name){
    output = createWriter(name);
  }
  
  public void printPressForce(Field pressure, BodyUnion bodies, float L){
    for(Body body: bodies.bodyList){
        PVector force = body.pressForce(pressure);
        output.print(2.f*force.x/L+" "+2.f*force.y/L+" ");
    }
    output.println();
  }

  public void close(){
    output.close(); // Finishes the file
  }  
}
/********************************
 Bread class by tree for fluid sensing
 simple version 1

Examlpe code: 

Body body;
void setup(){
  size(400,400);
  Window window = new Window(300,300);
  body = new Bread(70,70,1,1.5,4,window);
}
void draw(){
  background(0);
  body.follow();
  body.display();
}
void mousePressed(){body.mousePressed();}
void mouseReleased(){body.mouseReleased();}

 ********************************/


class Bread extends Body {
  int m = 20;//elipse and straight lines have both 80 points. We can modify the num later.
  float ratio = 3;//make it bigger
  Bread( float x, float y, float _h, float _l1, float _l2, int resolution, Window window) {
    super(x*resolution, y*resolution, window);
    float h= resolution * _h;
    float l1= resolution * _l1;
    float l2= resolution * _l2;
    for ( int i=0; i<m/2; i++ ) {//left ellipse
      float theta = -TWO_PI*i/((float)m);
      add(xc.x+l1*cos(theta-PI/2), xc.y+h/2*sin(theta-PI/2));
    }
    for ( int i=0; i<m/2; i++ ) {//down straight
      add(xc.x+l2*i/m, xc.y+h/2);
    }
    for ( int i=m/2; i<m; i++ ) {//right ellipse
      float theta = -TWO_PI*i/((float)m);
      add(xc.x+l2+l1*cos(theta-PI/2), xc.y+h/2*sin(theta-PI/2));
    }
    for ( int i=m/2; i<m; i++ ) {//up straight
      add(xc.x+l2*(1-PApplet.parseFloat(i)/m), xc.y-h/2);
    }
    end(); // finalize shape
  }
}



/************************
Chaotic Ellipse Class

This class demonstrates how the `react` function can be
used to model rigid-body fluid/structure interactions.

Example Code:

BDIM flow;
Body body;
FloodPlot flood;
int example = 3; // Choose an example reaction function

void setup(){
  size(700,700);                             // display window size
  int n=(int)pow(2,7);                       // number of grid points
  float L = n/4., l = 0.2;                   // length-scale in grid units
  Window view = new Window(n,n);
  body = new ChaoticEllipse(n/3,n/2,L*l,l,example,view); // define geom
  flow = new BDIM(n,n,1.,body);               // solve for flow using BDIM
  flood = new FloodPlot(view);                // intialize a flood plot...
  flood.setLegend("vorticity",-.5,.5);        //    and its legend
}
void draw(){
  body.react(flow);
  flow.update(body); flow.update2();         // 2-step fluid update
  flood.display(flow.u.curl());              // compute and display vorticity
  body.display();                            // display the body
}
*/

class ChaoticEllipse extends EllipseBody{
  float pivot, y0, x0, L;
  int example=0;
  
  ChaoticEllipse( float x, float y, float h, float a, float pivot, int example, Window window ){
    super(x+h/a*(0.5f-pivot),y,h,a,window);
    this.example = example;
    x0 = x; y0 = y; L = h/a;
    // move pivot to x and adjust properties
    xc.x = x; box.xc.x = x;
    I0 += sq(L*(0.5f-pivot))*area;
    ma.z += sq(L*(0.5f-pivot))*ma.y;
  }
  ChaoticEllipse( float x, float y, float h, float a, int example, Window window ){
    this(x,y,h,a,0.5f,example,window);
  }
  
  public void display(int C, Window window ){
    super.display( C, window );
    fill(255);
    ellipse(window.px(xc.x),window.py(xc.y),5,5);
  }

  // Specialized body reaction functions
  public void react(BDIM flow){
    if(example==1){
    // Example 1: free pitch, no actuation, no limits
      xfree = false; yfree=false; pfree=true;
      super.react(flow);
    } else if(example==2){
    // Example 2: free pitch, actuated heave, no limits
    //   Use the `react`-computed phi/dphi in `follow`
      xfree = false; yfree=false; pfree=true;
      super.react(flow);
      float St = 0.2f, amp = 0.5f*L;
      float y = amp*sin(TWO_PI*St/L*flow.t),
            dy = amp*cos(TWO_PI*St/L*flow.t)*TWO_PI*St/L*flow.dt;
      follow(new PVector(x0,y0+y,phi), new PVector(0,dy,dphi));  
    } else if(example==3){
    // Example 3: free pitch and heave, no actuation, heave limits
    //    Compute f.y and use to `check_y_free` and in `react`.
      PVector f = pressForce(flow.p).mult(-1);
      float m = pressMoment(flow.p)*(-1);
      xfree = false; yfree = check_y_free(f.y,y0+L,y0-L); pfree=true;
      super.react(f,m,flow.dt);
    } else {
      println("Unknown update example");
      stop();
    }
  }
}
/*********************************
Sets up an array of circles of the same diameter into a ringed arrangement.
The input arguments are center coordinates X, Y; diameter of each cylinder D; 
Radius of circle array; division n; Angle of Attack rad; window.

Example code: 

CircleArrangement body; 
BDIM flow;
FloodPlot flood;
int n = (int)pow(2,8), m = n/2;
float DG = m/2;            // Bundle diameter
float d = DG/21.;          // Cylinder's diameter
float ReG = 2100;          // physical Reynolds number

void setup(){
  size(1000,500);           // display window size
  float Reh = ReG/DG;       // grid-based Reynolds number
  float x = m/2, y = m/2.;  // central position 
  Window view = new Window(n,m);
  body = new CircleArrangement(x, y, d, DG/2, 20, PI/2, view); 
  flow = new BDIM(n,m,0,body,(float)1./Reh,true);
  flood = new FloodPlot(view);
  flood.setLegend("vorticity",-.75,.75);
}

void draw(){
  flow.update();
  flow.update2();
  flood.display(flow.u.curl());
  body.display();
}
**********************************/

class CircleArrangement extends BodyUnion {
  CircleArrangement(float x, float y, float d, float R, int n, float rad, Window window) {
    super(x, y, window);
    
    int N; // N is number of rings
    if (n<=7){
      N = 1;
    } else if (n<=20){
      N = 2;
    } else if (n<=39){
      N = 3;
    } else if (n<=65){
      N = 4;
    } else if (n<=96) {
      N = 5;
    } else{
      N = 6;
    }
    
//  New a arraylist to restore the number of maximum cylinders on each ring.
//  This set of numbers is implemented from Eames' paper.
    int[] ring = new int[7];
    ring[1] = 6;
    ring[2] = 13;
    ring[3] = 19;
    ring[4] = 25;
    ring[5] = 31;

//  The logic for the arrangement is expect from centric cylinder and most outer ring, 
//  the middle parts are arranged separately with specific AoA.
    if (N==1){
      if(n>1){
        ring( x, y, d, R, n-1, rad, window );
      }
      ring( x, y, d, 0, 1, rad, window );
    } else{
      int temp;
      temp = n-1;
      for (int i=1; i<N; i++){ //loop in the different ring; i is current ring.
        ring( x, y, d, R/N*i, ring[i], rad, window );
        temp = temp - ring[i];
      }
      ring( x, y, d, R, temp, rad, window );
      ring( x, y, d, 0, 1, rad, window );
    }
  }
  
  public void ring(float x, float y, float d, float R, int n, float rad, Window window){
    for ( int i = 1; i <= n; i++ ){
      float X = x+R*cos(TWO_PI/n*i+rad);
      float Y = y+R*sin(TWO_PI/n*i+rad);
      add(new CircleBody(X, Y, d, window));
    }    
  }
}
/**********************************
 CirculationFinder class
 
 Finds and displays circulation of vortices in the wake.
 Algorithm: 
 1) Finds local peaks in vorticity field, labelling them as circular cores
 2a) Grows the cores until the vorticity at the border dies out
 2b) If cores of the same sign interefere with each other, core with less circulation is removed (as defined a rough circulation estimate)
 3) Finds final circulation by adding the vorticity within the core; however, only adds the vorticity of a single sign
 example code:
 
 BDIM flow;
 Body body;
 FloodPlot flood;
 CirculationFinder cf;
 
 void setup(){
   int n=(int)pow(2,7);
   float d = n/12;
   size(800,400);
   Window view = new Window(n,n/2);
 
   body = new CircleBody(n/4,n/4,d,view);
   flow = new BDIM(n,n/2,1.5,body);
 
   cf = new CirculationFinder(flow,body,view);
   cf.setAnnotate(true,1.0/d);
 
   flood = new FloodPlot(view);
   flood.setLegend("vorticity",-0.5,0.5);
 }
 void draw(){
   flow.update();flow.update2();
   cf.update();
 
   flood.display(flow.u.curl());
   body.display();
   cf.display();
 }
 ***********************************/

class CirculationFinder {
  ArrayList<VortexCore> cores = new ArrayList(1024);
  ArrayList<VortexCore> toremove = new ArrayList(1024);
  Window window;
  Field vorticity;
  float circres=8, r_init=1, dr=2, r_max;
  BDIM flow;
  Body body;

  float Qmin, wmin, bmin, diatol, annoscale = 1;
  boolean dispG=true;

  CirculationFinder(BDIM flow, Body body, Window window) {
    this.flow = flow;
    this.body = body;
    this.window = window;
    setParams(0.01f, 0.01f, 5, 0.05f);
    r_max = 0.25f*window.dy;
  }
  public void setParams(float Qmin, float wmin, float bmin, float diatol) {
    this.Qmin = Qmin; //Minimum Qcriterion to be considered a core
    this.wmin = wmin; //Minimum vorticity in center to be considered a core
    this.bmin = bmin; //Minimum distance from body
    this.diatol = diatol; //Tolerance on vorticity reaching asymtote as the core diameter increases, as a fraction of center vorticity
  }
  public void setAnnotate(boolean dispG, float s) {
    this.dispG=dispG; //Display circulation value
    this.annoscale=s;  //Display scaling of circulation values
  }
  public void update() {
    //Updates the vortex cores

    // +++++++++++++++ Step 1 - Finds local maxima in vorticity that also satisfy requirements of minimum vorticity, Qcriterion, and distance from body
    vorticity = flow.u.curl();
    float[][] w = vorticity.a;
    cores.clear();
    for ( int i=1 ; i<flow.n-1 ; i++ ) {
      for ( int j=1 ; j<flow.m-1 ; j++ ) {
        //Find local peaks
        if ((w[i+1][j] < w[i][j] && w[i-1][j] < w[i][j] && w[i][j+1] < w[i][j] && w[i+1][j-1] < w[i][j]) || (w[i+1][j] > w[i][j] && w[i-1][j] > w[i][j] && w[i][j+1] > w[i][j] && w[i+1][j-1] > w[i][j])) {
          if (Qcrit(i, j, flow.u)>Qmin && abs(w[i][j])>wmin && body.distance((float)i, (float)j)>bmin) { //Check for Q criterion, max vorticity, and distance from body
            cores.add(new VortexCore(i, j, r_init, w[i][j])); //Add a new vortex core
          }
        }
      }
    }
    println("Number of Valid Peaks: " + cores.size());

    // +++++++++++++++ Step 2 - Expands all the cores until either they interefere with each other, or the vorticity dies out
    boolean running=true; //indicator that all cores have finished dilating
    while (running) {
      running = false;

      //Dilate all the cores, but only if the vorticity on the border is above a threshold
      for (VortexCore c : cores) {
        float meanborderw = lineinteg(c.xc, c.r, vorticity, true)/(2*PI*c.r); //Find mean of vorticity along perimeter
        if ((abs(meanborderw)>diatol*abs(c.w))) { //Check if mean vorticity along perimeter is greater than a small fraction of max vorticity
          if ((c.r<r_max)&&(c.r<body.distance(c.xc.x, c.xc.y)-dr)) { //Check if radius is too big or interferes with body
            //Dilate the given core, and reset indicator
            running = true;
            c.G_rough += meanborderw*(2*PI*c.r)*dr; //Rough circulation found iteratively, used to compare strength of vorticies
            c.r += dr;
          }
        }
      }

      //Check for core interference with other cores
      toremove.clear();
      for (int i=cores.size()-1; i>=0; i--) {
        VortexCore ci = cores.get(i);
        for (int j=i-1; j>=0; j--) {
          VortexCore cj = cores.get(j);
          //Check for interference between cores
          if ((PVector.sub(ci.xc, cj.xc).mag()<(ci.r+cj.r+2*dr)) && (cj.w*ci.w>0)) {       
            //Remove core with smaller rough circulation if same sign
            if (abs(ci.G_rough)>abs(cj.G_rough))
              toremove.add(cj);
            else
              toremove.add(ci);
          }
        }
      }
      cores.removeAll(toremove);
    }

    //+++++++++++++++ Step 3 - Find final circulation again by area method, ignoring vorticity of the wrong sign
    for (VortexCore c: cores)
      c.G = areainteg(c.xc, c.r, vorticity, true);
    println("Final Number of Cores: " + cores.size());
  }
  public void display() {
    //Displays the vortex cores, and labels the circulation if specified
    int G = 0xff000393, S = 0xff0003A3, w = 0xff0003C9, txtpnt = 12;
    PImage img = copy();
    img.loadPixels();
    if (cores.size()>0) {
      for ( VortexCore c: cores ) {
        stroke(img.pixels[window.px(c.xc.x)+window.py(c.xc.y)*img.width]);
        fill(0, 0);
        dashcirc(c.xc.x, c.xc.y, c.r, round(c.r*2), window);
        if (dispG) {
          textSize(txtpnt);
          textAlign(CENTER, TOP);
          fill(0);
          char sign = '+';
          if (c.G<0)
            sign = '-';
          text("" + (char)S + (char)w + sign + ": " + nfs(c.G*annoscale, 1, 2), window.px(c.xc.x), window.py(c.xc.y+c.r));
        }
      }
    }
  }
  public void dashcirc(float x0, float y0, float r, int n, Window window) {
    //Draws a dashed circle at (x0,y0) with radius 'r' and 'n' dashes
    float x1, y1, x2, y2;
    for (float i=0; i<2*n-1; i+=2) {
      x1 = r*cos(PI*i/n)+x0;
      y1 = r*sin(PI*i/n)+y0;
      x2 = r*cos(PI*(i+1)/n)+x0;
      y2 = r*sin(PI*(i+1)/n)+y0;
      line(window.px(x1), window.py(y1), window.px(x2), window.py(y2));
    }
  }
  public float lineinteg(PVector xc, float r, VectorField u) {
    //Finds line integral about a circle, integral of u-dot-ds
    int n = round(r*circres);
    float res=0, x, y, dx, dy, dth = 2*PI/n;
    if (r<0)
      return 0;
    for (int i=0; i<n; i++) {
      x = r*cos(i*dth);
      y = r*sin(i*dth);
      dx = x-r*cos((i-1)*dth);
      dy = y-r*sin((i-1)*dth);
      res += u.x.linear(xc.x+x, xc.y+y)*dx + u.y.linear(xc.x+x, xc.y+y)*dy;
    }
    return res;
  }
  public float lineinteg(PVector xc, float r, Field F, boolean sum_only_same_sign) {
    //Finds line integral about a circle, integral of Fds
    float n = round(r*circres);
    float res=0, x, y, dth = 2*PI/n, Fds;
    float Fmid = F.linear(xc.x, xc.y);
    if (r<0) {
      return 0;
    }
    for (int i=0; i<n; i++) {
      x = r*cos(i*dth);
      y = r*sin(i*dth);
      Fds = F.linear(xc.x+x, xc.y+y)*r*dth;
      if (!sum_only_same_sign || (Fds*Fmid>0 && sum_only_same_sign))
        res += Fds;
    }
    return res;
  }
  public float areainteg(PVector xc, float R, Field F, boolean sum_only_same_sign) {
    //Finds area integral on the inside of a circle, integral of FdA, using concentric circles
    float res=0, x, y, n, dth, r1, r2, FdA, Fmid; 
    if (R<0)
      return 0;
    Fmid = F.linear(xc.x, xc.y)*PI*dr*dr/4.0f;
    for (float r=1; r<=R; r+=dr) {
      n = round(r*circres);
      r2 = r+dr/2; 
      r1 = r-dr/2;
      if (r==R)
        r2 = R;
      dth = 2*PI/n;
      for (int i=0; i<n; i++) {
        x = r*cos(i*dth);
        y = r*sin(i*dth);
        FdA = F.linear(xc.x+x, xc.y+y)*(r2*r2-r1*r1)*dth/2.0f;
        if (!sum_only_same_sign || (FdA*Fmid>0 && sum_only_same_sign))
          res += FdA;
      }
    }
    return res+Fmid;
  }
  public float Qcrit(int i, int j, VectorField velo) {
    //Finds Qcriterion of flow at location (i,j)
    if ((i-1<0)||(j-1<0)||(i+1>velo.n-1)||(j+1>velo.m-1))
      return 0;
    float[][] u = velo.x.a;
    float[][] v = velo.y.a;
    float dudx = 0.5f*(u[i+1][j]-u[i-1][j]);
    float dudy = 0.5f*(u[i][j+1]-u[i][j-1]);
    float dvdx = 0.5f*(v[i+1][j]-v[i-1][j]);
    float dvdy = 0.5f*(v[i][j+1]-v[i][j-1]);
    return dudx*dvdy-dvdx*dudy;
  }
}
class VortexCore {
  //Vortex Core object, has two circulations 'G' and 'G_rough' (area integral of vorticity and a rough estimate used to compare vortex strengths)
  //Also has a center 'xc', radius 'r', vorticity at center 'w'
  float G_rough, G, r, w;
  PVector xc;
  boolean grow;
  VortexCore(float x, float y, float r, float w, float Gv, float G) {
    this.xc = new PVector(x, y, 0);
    this.r = r;
    this.w = w;
    this.G_rough = G;
    this.G = G;
    this.grow = true;
  }
  VortexCore(float x, float y, float r, float w) {
    this(x, y, r, w, 0, 0);
  }
  VortexCore(PVector xc, float r, float w) {
    this(xc.x, xc.y, r, w, 0, 0);
  }
}
/*********************************************************
CounterRotatingCylinders 

  Extends BodyUnion to create counter-rotating cylinders.  
  The position and size are defined with respect to a main 
  cylinder, D, located at (xc, yc).

  In the example you can adjust the spin rate with the up/down arrows keys.

Example Code:

BDIM flow;
CircleBody mainCylinder;
CounterRotatingCylinders controlCylinders;
BodyUnion body;
FloodPlot flood;

void setup(){

  // --input parameters-------
  int n=(int)pow(2,6);  // number of grid points along a side of domain
  float Re = 100;  // Reynolds number by main cylinder diameter
  float dR = 0.15;  // (Control cylinder diameter)/(Main cylinder diameter)
  float theta = 60*PI/180;  // Angular location of control cylinders from downstream stream stagnation point
  float gR = 0.1;  // (Gap between control and main cylinders)/(Main cylinder diameter)
  float D = n/2.5;  // Main cylinder diameter
  float xc = 2*D, yc = n; // Position of Main cylinder
  // -------------------------

  size(900,600);  // display window size
  Window view = new Window(3*n,2*n);
  
  // Create union of main cylinder and counter-rotating cylinders
  mainCylinder = new CircleBody(xc,yc,D,view);
  controlCylinders = new CounterRotatingCylinders(xc, yc, dR, theta, gR, 0, D, view);
  body = new BodyUnion(mainCylinder, controlCylinders);
  flow = new BDIM(3*n, 2*n, 0, body, D/Re, true);

  flood = new FloodPlot(view);
  flood.range = new Scale(-.25,.25);
  flood.setLegend("vorticity");
}

void draw(){
  controlCylinders.update(flow.dt);
  flow.update(body);
  flow.update2();
  flood.display(flow.u.curl());
  body.display();
  fill(0); text("Xi = " + controlCylinders.xi,width/2,height-30);
}

void keyPressed() {
  if (key == CODED) {
    if (keyCode == UP) {
      controlCylinders.xi += 1;
    } else if (keyCode == DOWN) {
      controlCylinders.xi -= 1;
    } 
  }
}
*********************************************************/

class CounterRotatingCylinders extends BodyUnion{
  float xi,d;

  CounterRotatingCylinders(float xc, float yc, float dR, float theta, float gR, float xi, float D, Window window) {
    super(xc,yc,window);                               // initiate BodyUnion
    d = dR*D;                                          // control cylinder diameter
    float l = D/2.f+gR*D+d/2.f;                          // center-to-center distance
    float dx = l*cos(theta), dy = l*sin(theta);        // x,y offset
    add(new CircleBody(xc+dx, yc+dy, d, window));      // add top cylinder
    add(new CircleBody(xc+dx, yc-dy, d, window));      // add bottom
    this.xi = xi;                                      // rotation rate
  }

// Rotate the control cylinders
  public void update(float dt){
    float omega = xi*2.f/d;
    this.bodyList.get(0).rotate(-omega*dt);
    this.bodyList.get(1).rotate( omega*dt);
  }
}
/*********************************************************
 Simulation of the Duncan experiment
 
 A foil at angle of attack below a recirculating channel with free surface. 
 
 Example code:
 
 Duncan test;
 
 void settings() {
 size(1200, 600);
 }
 
 void setup() {
 int n = 128, m=64;
 float xstart = 2, ystart=0.5, AoA=15, Re=500, Fr=0.5; 
 test = new Duncan( n,  m,  xstart,  ystart,  AoA,  Re,  Fr);
 }
 
 void draw() {
 test.update();
 test.display();
 }
 ***********************/

class Duncan {
  float density=0.001f, fill=.5f; // air/water density and domain fill ratios
  NACA wing;
  TwoPhase flow;
  PVector state;
  float t=0, U=1, L, y0, g;

  Duncan( int n, int m, float xstart, float ystart, float AoA, float Re, float Fr) {
    // set up wing
    L = m/4; 
    state = new PVector(xstart*L, fill*m + ystart*L, radians(AoA));
    wing = new NACA(0, 0, L, 0.16f, new Window(n, m));
    wing.follow(state, new PVector());
    wing.bodyColor=color(255);

    // set up two phase fluid
    float nu = U*L/Re;
    g = pow(U, 2)/pow(Fr, 2)/L;
    flow = new TwoPhase(n, m, 0, wing, nu, true, g, density); // define two phase fluid
    flow.f.eq(0, 0, n+2, 0, PApplet.parseInt(fill*m));                     // define initial water region
    y0 = PApplet.parseInt((1-fill)*flow.m+0.5f);
  }

  public void update() {
    t += flow.dt;   // update the time
    flow.update();  // 2-step fluid update
    flow.update2();
  }

  public void display() {
    flow.f.display();      // free surface
    flow.u.display(2, 2);  // velocity vectors
    wing.display();        // geometry
  }

  public void pressForce(PrintWriter w) {
    // measure pressure forces on wing and print to writer
    PVector forces = wing.pressForce(flow.p);
    float drag = 2.f*forces.x/L, lift = 2.f*forces.y/L, ts = t*U/L;
    w.println(""+ts+","+drag+","+lift+"");
  }

  public void height(PrintWriter w) {
    // measure fluid height for each x and print to writer
    for ( int i=1; i<flow.f.n-1; i++) {
      float h = 0;
      for ( int j=1; j<flow.f.m-1; j++) {
        h+=flow.f.a[i][j];
      }
      float x = (i-state.x)/L, y = (h-y0)/L, ts = t*U/L;
      w.println(""+ts+","+x+","+y);
    }
  }
}

class WaveyDuncan extends Duncan {
  float k, a, omega, omega_e;
  WaveyDuncan( int n, int m, float xstart, float ystart, float AoA, float Re, float Fr, float kL, float ak) {
    super(n, m, xstart, ystart, AoA, Re, Fr);
    //U=0; // no forward speed test !!
    flow.u = new VectorField(n+2, m+2, U, 0.f); // initial velocity field
    flow.u.x.waveInlet = true;                 // wave inlet
    flow.u.y.waveTop = true;                   // volume correcting top boundary
    k = kL/L;
    a = ak/k;
    omega = sqrt(g*k);                         // wave frequency
    omega_e = omega+U*k;                       // encounter frequency
  }

  public void update() {
    t += flow.dt;   // update the time
    set_BC();       // set inlet wave BCs
    flow.update();  // 2-step fluid update
    flow.update2();
  }

  public void set_BC() {
    float eta = a*cos(-omega_e*flow.t);  // inlet interface height
    float s = 0; // volume flux
    
    // apply wavemaker at inlet
    for (int j=1; j<flow.m-1; j++ ) {
      float y=flow.m-j-0.5f-y0;          // vertical position

      if (y<eta-0.5f) {       // under the wave
        float u = -omega*eta*exp(y*k);
        flow.u.x.a[1][j] = U+u;
        s+= u;
      } else if (y<=eta+0.5f) { // at the interface
        float f = eta-y+0.5f;
        float u = -f*omega*eta*exp(y*k);
        flow.u.x.a[1][j] = U+u;
        s+= u;
      } else {                // above the wave
        flow.u.x.a[1][j] = U;
      }
    }
    
    // apply uniform volume flux correction to upper boundary
    flow.u.y.bval_top = -s/PApplet.parseFloat(flow.n-2);
  }
}
/**********************************
Field class

Holds the values and operators for a
fluid dynamics field

example code:
Field p,u,v;

void setup(){
  size(400,400);
  background(1,0,0);
  p = new Field(100,150);
  p.eq(1.0,40,50,40,75);
  u = new Field(100,150,1,0.25);
  v = new Field(100,150,2,0.2);
}
void draw(){
  p.advect( 1., u, v );
  p.display(0,1);
}
***********************************/

class Field{
  float[][] a;  
  int n,m,btype=0;
  float bval=0,bval_top=0;
  boolean gradientExit=false,waveInlet=false,waveTop=false;
  
  Field( int n, int m, int btype, float bval ){
    this.n = n;
    this.m = m;
    this.a = new float[n][m];
    this.btype = btype;
    this.bval = bval;
    this.eq(bval);
  }
  Field( int n, int m){this(n,m,0,0);}

  Field( Field b, int is, int n, int js, int m ){
    this.n = n;
    this.m = m;
    a = new float[n][m];
    btype = b.btype;
    bval = b.bval;
    for( int i=0; i<n; i++){
    for( int j=0; j<m; j++){
      a[i][j] = b.a[i+is][j+js];
    }}
  }
  Field( Field b ){
    n = b.n;
    m = b.m;
    a = new float[n][m];
    btype = b.btype;
    bval = b.bval;
    this.eq(b);
  }

  public Field laplacian (){
    Field d = new Field( n, m ); 
    d.btype = btype;
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      d.a[i][j] = -4*a[i][j]+a[i+1][j]
        +a[i-1][j]+a[i][j+1]+a[i][j-1];
    }}
    return d;
  }

  public VectorField gradient(){
    mismatch(btype,0);
    VectorField g = new VectorField(n,m,0,0);
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      g.x.a[i][j] = a[i][j]-a[i-1][j];
      g.y.a[i][j] = a[i][j]-a[i][j-1];
    }}
    g.setBC(); // issues?
    return g;
  }
  
  public VectorField curl (){
    // returns curl{this \hat z}
    mismatch(btype,3);
    VectorField g = new VectorField(n,m,0,0);
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      g.x.a[i][j] = a[i][j+1]-a[i][j];
      g.y.a[i][j] = a[i][j]-a[i+1][j];
    }}
    return g;
  }

  public void advect( float step, VectorField u, VectorField u0 ){
    advect( step, u.x, u.y, u0.x, u0.y );
  }
  public void advect( float step, Field u, Field v, Field u0, Field v0 ){
    /* advect the field with the u,v velocity field
       here we use a first order lagrangian method
         Da/Dt = 0
         a(t=dt,x,y) = a(t=0,x-dt*u(x,y),y-dt*v(x,y))
       the example code shows how diffusive this is 
       EDIT: by using an RK2 step to find x0 and using
       a quadratic interpolation, this method is second
       order and nondiffuse.
       EDIT2: by treating the old and new velocities 
       seperately, the method is second order in time.*/
    Field a0 = new Field(this);
    for( int i=1; i<n-1; i++){
      for( int j=1; j<m-1; j++){
        float x = i;
        float y = j;
        if(btype==1) x -= 0.5f;
        if(btype==2) y -= 0.5f;
        float ax = -step*u.linear( x, y );
        float ay = -step*v.linear( x, y );
        float bx = -step*u0.linear( x+ax, y+ay );
        float by = -step*v0.linear( x+ax, y+ay );
        a[i][j] = a0.quadratic( x+0.5f*(ax+bx), y+0.5f*(ay+by) );
      }
    }
    setBC();
  }
  public void advect( float step, VectorField u ){
    advect( step, u.x, u.y );
  }
  public void advect( float step, Field u, Field v){
    /* advect the field with the u,v velocity field
       here we use a first order lagrangian method
         Da/Dt = 0
         a(t=dt,x,y) = a(t=0,x-dt*u(x,y),y-dt*v(x,y))
       the example code shows how diffusive this is 
       EDIT: by using an RK2 step to find x0 and using
       a quadratic interpolation, this method is second
       order and nondiffuse.
       EDIT2: RK2 step is not needed for first step
       in time accurate soln.*/
    Field a0 = new Field(this);
    for( int i=1; i<n-1; i++){
      for( int j=1; j<m-1; j++){
        float x = i;
        float y = j;
        if(btype==1|btype==3) x -= 0.5f;
        if(btype==2|btype==3) y -= 0.5f;
        float ax = -step*u.linear( x, y );
        float ay = -step*v.linear( x, y );
        a[i][j] = a0.quadratic( x+ax, y+ay );
      }
    }
    setBC();
  }

  public float quadratic( float x0, float y0){
    float x = x0, y = y0;
    if(btype==1|btype==3) x += 0.5f;
    if(btype==2|btype==3) y += 0.5f;
    int i = round(x), j = round(y);
    if( i>n-2 || i<1 || j>m-2 || j<1 )
      return linear( x0, y0 );
    x -= i; y -= j;
    float e = quadratic1D(x,a[i-1][j-1],a[i][j-1],a[i+1][j-1]);
    float f = quadratic1D(x,a[i-1][j  ],a[i][j  ],a[i+1][j  ]);
    float g = quadratic1D(x,a[i-1][j+1],a[i][j+1],a[i+1][j+1]);
    return quadratic1D(y,e,f,g);
  }
  public float quadratic1D(float x, float e, float f, float g){
    float x2 = x*x;
    float fx = f*(1.f-x2);
    fx += (g*(x2+x)+e*(x2-x))*0.5f;
    fx = min(fx,max(e,f,g));
    fx = max(fx,min(e,f,g));
    return fx;
  }  
  public float linear( float x0, float y0 ){
    float x  = min(max(0.5f,x0), n-1.5f);
    if(btype==1|btype==3) x += 0.5f;
    int i = min( (int)x, n-2 ); 
    float s = x-i;
    float y  = min(max(0.5f,y0), m-1.5f);
    if(btype==2|btype==3) y += 0.5f;
    int j = min( (int)y, m-2 );
    float t = y-j;
    if(s==0 && t==0){
      return a[i][j];
    }else{
      return s*(t*a[i+1][j+1]+(1-t)*a[i+1][j])+
         (1-s)*(t*a[i  ][j+1]+(1-t)*a[i  ][j]);
    }
  }
  public float interp( float x0, float y0 ){return linear(x0,y0);}
  
  public void display( float low, float high ){
    PImage img = createImage(n-2,m-2,RGB);
    img.loadPixels();
    for ( int i=0 ; i<n-2 ; i++ ) {
      for ( int j=0 ; j<m-2 ; j++ ) {
        float f = a[i+1][j+1];
        int k = i+j*(n-2);
        f = map(f,low,high,0,255);
        img.pixels[k] = color(f);
      }
    }
    img.updatePixels();
    int x0 = width/(n-2), y0 = height/(m-2);
    image(img,x0,y0,width,height);
  }

  public void setBC (){
    float s=0;
    for (int j=0 ; j<m ; j++ ) {  
      a[0][j]   = a[1][j];  
      a[n-1][j] = a[n-2][j];      
      if(btype==1){
        if(gradientExit || waveInlet){
          if(gradientExit) a[1][j] = bval;
          if(j>0 & j<m-1) s += a[n-1][j];          
        } else {
          a[1][j]   = bval;  
          a[n-1][j] = bval;
    }}}
    for (int i=0 ; i<n ; i++ ) {  
      a[i][0]   = a[i][1];
      a[i][m-1] = a[i][m-2];
      if(btype==2){
        if(waveTop){
          a[i][1] = bval_top;
        }else{
          a[i][1] = bval;
        }
        a[i][m-1] = bval;
      }   
    }
    if(gradientExit || waveInlet){
      s /= PApplet.parseFloat(m-2);
      for( int j=1; j<m-1; j++ ) a[n-1][j] += bval-s;
    }
  }
  
  public Field normalGrad(VectorField wnx, VectorField wny){
    Field g = new Field(n,m,0,0);
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      g.a[i][j] = 0.5f*(wnx.x.a[i][j]*(a[i+1][j]-a[i-1][j])+wny.x.a[i][j]*(a[i][j+1]-a[i][j-1]));
    }}
    return g;
  }

  public Field times( float b ){
    Field c = new Field(this);
    for( int i=0; i<n; i++){
    for( int j=0; j<m; j++){
        c.a[i][j] *= b;
    }}
    return c;
  }
  public Field times( Field b ){
    mismatch(this.btype,b.btype); 
    Field c = new Field(this);
    for( int i=0; i<n; i++){
    for( int j=0; j<m; j++){
      c.a[i][j] *= b.a[i][j];
    }}
    return c;
  }
  public void timesEq( float b ){ eq(times(b)); }
  public void timesEq( Field b ){ eq(times(b)); }
  public Field plus( float b ){
    Field c = new Field(this);
    for( int i=0; i<n; i++){
    for( int j=0; j<m; j++){
      c.a[i][j] += b;
    }}
    return c;
  }
  public Field plus( Field b ){
    mismatch(this.btype,b.btype); 
    Field c = new Field(this);
    for( int i=0; i<n; i++){
    for( int j=0; j<m; j++){
      c.a[i][j] += b.a[i][j];
    }}
    return c;
  }
  public void plusEq( Field b ){ eq(plus(b)); }
  public void plusEq( float b ){ eq(plus(b)); }
  public Field minus( Field b ){
    mismatch(this.btype,b.btype); 
    Field c = new Field(this);
    for( int i=0; i<n; i++){
    for( int j=0; j<m; j++){
      c.a[i][j] -= b.a[i][j];
    }}
    return c;
  }
  public void minusEq( Field b ){ eq(minus(b)); }
  public Field inv(){
    Field c = new Field(this);
    for( int i=0; i<n; i++){
    for( int j=0; j<m; j++){
      c.a[i][j] = 1.f/c.a[i][j];
    }}
    return c;
  }
  public void invEq(){ eq(inv()); }
  public float inner( Field b ){
    mismatch(this.btype,b.btype); 
    double s = 0;
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      s += a[i][j]*b.a[i][j];
    }} 
    return (float)s;
  }
  public float sum(){
    float s = 0;
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      s += a[i][j];
    }} 
    return s;
  }
  public void eq( float b ,int is, int ie, int js, int je){
    for( int i=is; i<ie; i++){
    for( int j=js; j<je; j++){
      a[i][j] = b;
    }}
  }
  public void eq( float b){eq(b,0,n,0,m);}
  public void eq( Field b){
    for( int i=0; i<n; i++){
    for( int j=0; j<m; j++){
      a[i][j] = b.a[i][j];
    }}
  }
  
  public void mismatch(int i, int j){
    if(i!=j){
      println("You can't add or multiple fields of different types.");
      exit();
    }     
  }
  
  public float L_inf(){
    float mx = 0;
    for( int i=0; i<n; i++){
    for( int j=0; j<m; j++){
      mx = max(mx,abs(a[i][j]));
    }}
    return mx;
  }
}
class Fish3 {
  float dt,dtheta1,dtheta2,dtheta3;
  float Fd=0, Fd_dot=0, F, Fold;

  /////Ini_global
  float Re_test;//Re=6000
  float St_test;//Str=0.45
  int resolution_test; //final int Chord_lengthbyY = 8;//gird resolution
  
  /////Input_end
  float nu = resolution_test/Re_test;
  //int n=resolution_test * Chord_lengthbyY;//all resolution in direction Y 
  int n;

  /////Ini_endB
  BDIM flow;
  Part_head joint1;
  Part joint2;
  Part_tail joint3;
  BodyUnion fish3;
  FloodPlot flood;
  Window window;
  // float time=0;
  float t = 0;
  


  Fish3( int resolution, float Re, float st, boolean QUICK) {
    Re_test = Re;
    St_test = st;
    resolution_test = resolution;
    n=resolution_test * 4;//all resolution 
    
    window = new Window(2*n, n);
    
    nu = resolution_test/Re_test;
    

    //Fxcount=0; Pinputcount=0;
    time=0;
 
    joint1 = new Part_head(1.f*resolution,2.f*resolution,1.f*resolution,0.12f,window);
    joint2 = new Part(2.f*resolution,2.f*resolution,1.f*resolution,0.12f,window);
    joint3 = new Part_tail(3.f*resolution,2.f*resolution,1.f*resolution,0.12f,window);
    //joint2.rotate_init(Max_theta*sin(Delta_theta));
    fish3 = new BodyUnion(joint1, joint2);
    fish3.add(joint3);
    flow = new BDIM(2*n,n,0,fish3,(float)resolution_test/Re_test,true);
    //int PInumberForOutput = int(PI/Max_theta);

    


    flood = new FloodPlot(window);
    flood.range = new Scale(-.5f,.5f);
    flood.setLegend("vorticity");
    flood.setColorMode(1);
  }
  /*Fish3( int resolution, int xStart, float zoom, int Re) {
    this(resolution, 100, 50, xStart, zoom, Re, 1);
    window = new Window(1000, 500);
  }*/
  
  
  

//修改了updatePosition--增加了action接口
  public void updatePosition(float dt, float dalpha, float theta, float alpha, int flag){
    
    if(flag==0){
      print("");
    }
    else if(flag==1){
      if(time>=0){
        // connection(joint2, omega_c(.2, 0.3, 0), joint1);//rotate_dalpha, theta0, alpha0
        // connection(joint3, omega_c(-.2, 0.3, 0), joint2);
        connection(joint2, omega_c(dalpha, theta, alpha), joint1);//rotate_dalpha, theta0, alpha0
        connection(joint3, omega_c(-dalpha, theta, alpha), joint2);
        println("adjusting...");
      }
      // joint1.rotate(Max_theta*Rotate_omega*cos(Rotate_omega*time)*flow.dt);
      // joint2.rotate(Max_theta*Rotate_omega*cos(Rotate_omega*time+Delta_theta)*flow.dt);
      // dtheta1=Max_theta*Rotate_omega*cos(Rotate_omega*time)*flow.dt;
      // dtheta2=Max_theta*Rotate_omega*cos(Rotate_omega*time+Delta_theta)*flow.dt;
      
    }
    else if(flag==2){
      print(114514);
    }
  }
  // void updatePosition(float dt){
  //   updatePosition(dt, 1);
  // }

  //修改了update--增加了action接口
  public void update(float dalpha, float theta, float alpha) {
    if (flow.QUICK) {
      dt = flow.checkCFL();
      flow.dt = dt;//modify
    }
    updatePosition(dt,dalpha,theta,alpha,1);//修改
    flow.update(fish3);flow.update2();
    t += dt;
  }
  public void display() {
    flood.display(flow.u.curl());
    fish3.display();
  }
  public void connection(Body joint, float omega, float vx, float vy){
    joint.rotate(omega*flow.dt);
    joint.translate((-sin(joint.phi)*resolution*0.25f*omega+vx)*flow.dt, 
                    (cos(joint.phi)*resolution*0.25f*omega+vy)*flow.dt);
    //joint.rotate(omega3*flow.dt);
  }
  public void connection(Body joint,float omega){
    connection(joint, omega,0,0);
  }
  public void connection(Body joint,float omega, Body joint0){
    float vx=joint0.velocity(1,dt,joint0.coords.get(100).x,joint0.coords.get(100).y);
    float vy=joint0.velocity(2,dt,joint0.coords.get(100).x,joint0.coords.get(100).y);
    connection(joint, omega, vx, vy);
    float delta_x = joint0.coords.get(100).x - joint.coords.get(1).x;
    float delta_y = joint0.coords.get(100).y - joint.coords.get(1).y;
    //println(delta_x,delta_y);
    joint.translate(delta_x,delta_y);
  }
  public float omega_c (float rotate_dalpha, float theta0, float alpha0){
    return theta0*rotate_dalpha*cos(rotate_dalpha*time+alpha0);
  }
  public float omega_c (float rotate_dalpha, float theta0){
    return omega_c(rotate_dalpha, theta0, 0);
  }
}
/*******************************************

FlexNACA: NACA foil with superimposed traveling wave motions

Adding the wave to the body makes it non-convex and adds divergence to the body velocity.
Therefore a second geom "orig" is used to hold the original NACA coords.
Transformations are applied to the distance and normal calculations on the convex original geom.
The divergence free wave velocity is use for the body velocity field.

example code:
BDIM flow;
FlexNACA fish; 
FloodPlot flood;
float time=0;
float[] a={0,.2,-.1};
void setup(){
  int n=(int)pow(2,6);
  size(400,400);      
  Window view = new Window(n,n);

  fish = new FlexNACA(n/4,n/2,n/3,0.20,0.25,1.2,1.,a,view);
  flow = new BDIM(n,n,0.5,fish,0.001,true);
  flood = new FloodPlot(view);
  flood.range = new Scale(-.5,.5);
  flood.setLegend("vorticity");
  flood.setColorMode(1); 
}
void draw(){
  time += flow.dt;
  fish.update(time);
  flow.update(fish);
  flow.update2();
  flood.display(flow.u.curl());
  fish.display();
}

********************************/
class FlexNACA extends NACA{
  float k,omega,T,x0,time=0;
  float[] a;
  NACA orig;

  FlexNACA( float x, float y, float c, float t, float St, float vp, float _k, float[] _a, Window window ){

// set the NACA coords and save as orig
    super( x, y, c, t, window );
    orig = new NACA( x, y, c, t, window );

// set the wave parameters
    x0 = x-0.25f*c;
    k = _k*TWO_PI/c;
    omega = vp*k;
    T = TWO_PI/omega;

// set the amplitude envelope    
    a = _a;
    float s = 0; 
    for(float ai: a) s += ai;
    if(s==0) {s=1; a[0]=1;}
    for(int i=0; i<a.length; i++) a[i] *= St*T/s;

// add wave to the coords
    for ( PVector xx: coords ) xx.y += h(xx.x);
  }
  
  public float distance( float x, float y ){ // shift y and use orig distance
    return orig.distance( x, y-h(x) ); 
  }
  public PVector WallNormal(float x, float y  ){ // shift y and adjust orig normal
    PVector n = orig.WallNormal(x,y-h(x));
    n.x -= dhdx(x)*n.y;
    float m = n.mag();
    if(m>0) return PVector.div(n,m);
    else return n;
  }
  public float velocity( int d, float dt, float x, float y ){ // use wave velocity
    float v = super.velocity(d,dt,x,y);
    if(d==1) return v;
    else return v+hdot(x); 
  }
  
  public void translate( float dx, float dy ){ // translate both geoms and wave
    super.translate(dx,dy);
    orig.translate(dx,dy);
    x0+=dx;
  }
  public void rotate( float dphi ){} // no rotation

  public void update(float time) { // update time and coords
    for ( int i=0; i<coords.size(); i++ ) coords.set(i,orig.coords.get(i).copy());
    this.time = time;
    for ( PVector x: coords ) x.y += h(x.x);
    getOrth();
  }
  public boolean unsteady(){return true;}
  

/* The wave is given by h = A(x)*sin(k*x-omega*t)
   The amplitude envelope is polynomial. 
   The time and space derivatives are needed for the transforms above. */
  public float Ax( float x){
    float amp = a[0];
    for (int i=1; i<a.length; i++) amp += a[i]*pow((x-x0)/c,i);
    return amp;
  }
  public float arg( float x)   { return k*(x-x0)-omega*time; }
  public float h( float x )    { return Ax(x)*sin(arg(x)); }
  public float hdot( float x ) { return -Ax(x)*omega*cos(arg(x)); }
  public float dhdx( float x ) { // = A k cos + dA/dx sin
    float amp = a[1]/c; // dAdx
    for (int i=2; i<a.length; i++) amp += a[i]*(float)i/c*pow((x-x0)/c,i-1);
    return Ax(x)*k*cos(arg(x))+amp*sin(arg(x)); 
  }  
  
  
}
///*******************************************

//FlexNACA: NACA foil with superimposed traveling wave motions

//Adding the wave to the body makes it non-convex and adds divergence to the body velocity.
//Therefore a second geom "orig" is used to hold the original NACA coords.
//Transformations are applied to the distance and normal calculations on the convex original geom.
//The divergence free wave velocity is use for the body velocity field.

//example code:
/*
BDIM flow;
 //FlexNACA fish;
 Flexfish fish; 
 FloodPlot flood;
 float time=0;
 float[] a={0,.2,-.1};
 float dt = 0.5;
 
 void setup(){
 int n=(int)pow(2,7);
 size(400,400);      
 Window view = new Window(n,n);
 
 fish = new Flexfish(n/4,n/2,n/3,32,0.15,0.15,view);
 //fish = new FlexNACA(n/4,n/2,n/3,0.20,0.25,1.2,1.,a,view);
 flow = new BDIM(n,n,dt,fish,0.001,true);
 flood = new FloodPlot(view);
 flood.range = new Scale(-1,1);
 flood.setLegend("vorticity");
 flood.setColorMode(1); 
  
 }
 void draw(){
   //print(fish.translate( 1.0, 1.0 ));
 time += flow.dt;
 fish.update(time);
 //print(fish.distance(10.0,10.0),"h",fish.h(10.0));
 flow.update(fish);
 flow.update2();
 flood.display(flow.u.curl());
 fish.display();
 //for (float i=40; i<50;i++)print(fish.velocity(2,0.5,i,120),"a");
 }
 */

//********************************/
class Flexfish extends NACA {
  float k, omega, T, x0, alpha, lamda, time=0, a0, xxx=10.0f,fL,wf;
  ArrayList<PVector> tcoords=new ArrayList<PVector>();
  float [] xm=new float [2*m];
  float dt = 0.5f;
  
  NACA orig;
     

  Flexfish( float x, float y, float _fL, float _T, float a1, float _wf, Window window ) {
  
    // set the NACA coords and save as orig
    super( x, y, _fL, _wf, 0,window );
    orig = new NACA( x, y, _fL,_wf,0, window );

    // set the wave parameters
    a0 = a1;
    alpha = -0.5f;
    lamda = 1;
    T=_T;
    fL=_fL;
    wf=_wf;
     //add wave to the coords
    //for ( PVector xx: coords ) print(xx);
      //print("xx",coords);
      for ( int i=0; i<coords.size(); i++) {
      PVector poi = new PVector(coords.get(i).x-si((coords.get(i).x-xc.x)/fL)*(orig.coords.get(i).y-xc.y),    
                                xc.y+w((coords.get(i).x-xc.x)/fL)*fL-co((coords.get(i).x-xc.x)/fL)*(orig.coords.get(i).y-xc.y));
      coords.set(i, poi);
    }
      for (int i=0; i<coords.size(); i++) {
    tcoords.add(new PVector(0,0));
    }
  }


//question is from here
  public float distance( float x, float y ) { // shift y and use orig distance
    float dis = 1e10f;
      for( OrthoNormal o: orth ) dis = min(dis,o.distance(x,y,false));
      return (wn(x,y)==0)?dis:-dis; 
    //return orig.distance( x-si((x-xc.x)/fL)*wf*fL*offset((x-xc.x)/fL) , y);
    
  }
  
  public PVector WallNormal(float x, float y  ) { // shift y and adjust orig normal
    PVector wnormal = new PVector(0, 0);
    float dis = -1e10f;
    float dis2 = -1e10f;
 
    // check distance to each line, choose max
    for ( OrthoNormal o : orth ) {
      dis2=o.distance(x, y);
      if (dis2>dis) {
        dis=dis2;
        wnormal.x=o.nx;
        wnormal.y=o.ny;
      }
    }
    return wnormal;
  }
  
    
  
  public float velocity( int d, float dt, float x, float y ) { // use wave velocity
  
    float y1=xc.y, ty1=xc.y;//??????????
    float v = super.velocity(d,dt,x,y);
   
   
  
    if(d==1) {
     
      
      if((x<=xc.x)||(x>xc.x+fL)) v= v+0;
      else {
        if(y>xc.y+w((x-xc.x)/fL)*fL){
         for ( int i=1; i<coords.size()/2+1; i++ ){
         if ((coords.get(i-1).x<x)&&(x<=coords.get(i).x)){
           v=v+0;//vx(i,x)/e;//(coords.get(i).x-tcoords.get(i).x)/dt/e;
          }
         }
        }
        else{
          for ( int i=coords.size()/2+1; i<coords.size()+1; i++ ){
         if ((coords.get(i-1).x>=x)&&(x>coords.get(i%coords.size()).x)){
           v= v+0;//vx(i,x)/e;//(coords.get(i%coords.size()).x-tcoords.get(i%coords.size()).x)/dt/e;
           //print("w",xc.y+w((x-xc.x)/fL)*fL,"x",x,"y",y,"v",v);
          }
         }
        }
         
      } 
    }
    else {
      if((x<=xc.x)||(x>xc.x+fL)) v=0+ v;
      else{
     
        if(y>xc.y+w((x-xc.x)/fL)*fL) {
          for ( int i=1; i<coords.size()/2+1; i++ ){
         if ((coords.get(i-1).x<x)&&(x<=coords.get(i).x)) {
           y1=yloc(i,x);
         }
          }
          for ( int i=1; i<coords.size()/2+1; i++ ){
         if ((tcoords.get(i-1).x<x)&&(x<=tcoords.get(i).x)) {
           ty1=tyloc(i,x);
         }
         }
         //v=v+(y1-ty1)/dt;
         v=(y1-ty1)/dt;
         println("Vy = ", v, " X = ", x, " y = ", y, " T = ", time);
        }
        
        else{
          for ( int i=coords.size()/2+1; i<coords.size()+1; i++ ){
            if ((coords.get(i-1).x>=x)&&(x>coords.get(i%coords.size()).x)){
              y1=yloc(i,x);
            }
          }
          for ( int i=coords.size()/2+1; i<coords.size()+1; i++ ){
            if ((tcoords.get(i-1).x>=x)&&(x>tcoords.get(i%tcoords.size()).x)) {
              ty1=tyloc(i,x);
            }
          }
          //v=v+(y1-ty1)/dt; 
          v=(y1-ty1)/dt;  
        }      
      }
    }
 
    return v;
  }

  //void translate( float dx, float dy ) { // translate both geoms and wave
  //  super.translate(dx, dy);
  //  orig.translate(dx, dy);
  //  x0+=dx;
  //}
  //void rotate( float dphi ) {
  //} // no rotation

//till here

  public void update(float time) { // update time and coords
    for ( int i=0; i<coords.size(); i++ ) {
      coords.set(i, orig.coords.get(i).copy());
      //print(orig.coords.get(i).copy());
    }
    this.time = time;

    for ( int i=0; i<coords.size(); i++) {
      PVector poi = new PVector(coords.get(i).x-si((coords.get(i).x-xc.x)/fL)*(orig.coords.get(i).y-xc.y),    
                                xc.y+w((coords.get(i).x-xc.x)/fL)*fL-co((coords.get(i).x-xc.x)/fL)*(orig.coords.get(i).y-xc.y));
      coords.set(i, poi);
      //print(coords,"a");
    }
   
    getOrth();
    //print(coords);
    

    
    if (time>0){
    this.time = time-dt;
    }
    else this.time=time;
     for ( int i=0; i<coords.size(); i++) {
      PVector poi = new PVector(coords.get(i).x-si((coords.get(i).x-xc.x)/fL)*(orig.coords.get(i).y-xc.y),    
                                xc.y+w((coords.get(i).x-xc.x)/fL)*fL-co((coords.get(i).x-xc.x)/fL)*(orig.coords.get(i).y-xc.y));
      tcoords.set(i, poi);
      //print(coords,"a");
    }
    
    
    //print(coords);
  }
  
  public boolean unsteady() {
    return true;
  }




  public float w( float x){
    return Y(x)*sin(TWO_PI*x/lamda-time/T*TWO_PI);
    
  }

  public float Y( float x){
    return ae(x)*exp(-alpha)*abs((exp(alpha*x)-1));
  }
  public float ae(float x){
    return a0*exp(alpha)/abs(exp(alpha)-1);
  }
  public float dwdx( float x){
    return a0/abs(exp(alpha)-1)*(alpha*exp(alpha*x)*sin(TWO_PI/lamda*x-time/T*TWO_PI)+(exp(alpha*x)-1)*cos(TWO_PI/lamda*x-time/T*TWO_PI)*TWO_PI/lamda);
  }
  public float si( float x){
    return -sin(atan(dwdx(x)));
  }
  public float co( float x){
    return  -cos(atan(dwdx(x)));
  }
  public float dwdt( float x){
    return -Y(x)*cos(TWO_PI*x/lamda-time/T*TWO_PI)*TWO_PI/T;
  }
  public float offset( float x ){
    return 5*(0.2969f*sqrt(x)-0.1260f*x-0.3516f*pow(x,2)+0.2843f*pow(x,3)-0.1015f*pow(x,4));
  }
  public float yloc(int i, float x){
  return (coords.get(i).y-coords.get(i-1).y)*(x-coords.get(i-1).x)/(coords.get(i).x-coords.get(i-1).x)+coords.get(i-1).y;
  }
  public float tyloc(int i, float x){
  return (tcoords.get(i).y-tcoords.get(i-1).y)*(x-tcoords.get(i-1).x)/(tcoords.get(i).x-tcoords.get(i-1).x)+tcoords.get(i-1).y;
  }
  public float vx(int i, float x){
  return (x-(x-coords.get(i-1).x)/(coords.get(i).x-coords.get(i-1).x)*(tcoords.get(i).x-tcoords.get(i-1).x)+tcoords.get(i-1).x)/dt;
  }
}
  
/**********************************
 FloodPlot class
 
 Holds the values and operators for
 very nice flood plots
 
 example code:
void setup(){
  size(400,400);
  int n=100,m=2;
//  FloodPlot plot = new FloodPlot(new Window(1,1,n-2,m-2,100,1,300,325)); //custom window
  FloodPlot plot = new FloodPlot(new Window(n,m)); // standard window
  Field p = new Field(n,m);
  for( int i=0; i<n; i++){
  for( int j=0; j<m; j++){
    p.a[i][j] = 1.5*(0.5-i/(float)(n-1));
  }}
  plot.range = new Scale(-1,1);
  plot.hue = new Scale(200,140);
  plot.setLegend("test");
  plot.display(p);
}
***********************************/
 
class FloodPlot{
  LegendPlot legend;
  PImage img;
  Window window;
  Scale range = new Scale(0,1);
  Scale hue = new Scale(240,0);
  Scale sat = new Scale(1,1);
  Scale bri = new Scale(1,1);
  boolean sequential=false,legendOn=false,dark=false,sharp=false;

  FloodPlot( Window window, String title, float low, float high ){
    this.window = window;
    img = new PImage(window.dx,window.dy,HSB);
    setLegend(title, low, high);
  }
  FloodPlot( Window window ){
    this.window = window;
    img = new PImage(window.dx,window.dy,HSB);
  }
  FloodPlot( FloodPlot a){
    legend = a.legend; 
    img = a.img; 
    window = a.window;
    range = a.range; 
    hue = a.hue; sat = a.sat; bri = a.bri;
    sequential = a.sequential; 
    legendOn = a.legendOn; dark = a.dark; sharp = a.sharp;
  }

  public int colorScale( float f ){
    float i = range.in(f); // get % of the range
    if( sharp & i>0.4f & i<0.6f ) i = 0.5f;
    if(sequential){          // blend hue and saturation
      return color(hue.outB(i),sat.out(i),bri.out(i));
    } 
    else if(i<0.5f) {       // blend from low end to black/white
      if (dark) {return color(hue.outS,sat.outS,1-2*i);}
      else {return color(hue.outS,(0.5f-i)*2,1);}
    } 
    else {                 // blend from high end to black/white
      if (dark) {return color(hue.outE,sat.outE,2*i-1);}
      else {return color(hue.outE,(i-0.5f)*2,1);}
    }
  } 

  public void display ( Field a ){
    float minv = 1e6f, maxv = -1e6f;
    colorMode(HSB,360,1.0f,1.0f,1.0f);
    noStroke();
    img.loadPixels();
    for ( int i=0 ; i<window.dx ; i++ ) {
      float x = window.ix(i+window.x0);
      for ( int j=0 ; j<window.dy ; j++ ) {
        float y = window.iy(j+window.y0);
        float f = a.interp(x,y);
        img.pixels[i+j*window.dx] = colorScale(f);
        minv = min(minv,f);
        maxv = max(maxv,f);
      }
    }
    img.updatePixels();
    image(img,window.x0,window.y0);
    if(legendOn) legend.display(minv,maxv);
  }

  public void setLegend(String title, float low, float high){
    range = new Scale(low,high);
    legend = new LegendPlot(this,title);
    legendOn = true;
  }
  public void setLegend(String title){ setLegend( title, range.outS, range.outE );}

  public void setColorMode(int mode){
    if (mode==1){dark = false;}
    if (mode==2){sequential = true;}
    if (mode==3){sequential = true; sat = new Scale(0,0); bri = new Scale(0,1);}
    if (mode==4){sequential = true; sat = new Scale(1,0); bri = new Scale(0.24f,1); hue.outE = hue.outS; hue.r = 0;}
    if (legendOn){legend.setColorMode(mode);}
  }

  class LegendPlot extends FloodPlot{
    String title;
    Field levels;
    boolean box = false;

    LegendPlot( FloodPlot a, String _title){
      super(a); legendOn = false; // duplicate the flood settings, except for legend

      int n = 11, m = 2; // resize the window and image
      window = new Window(1,1,n-2,m-2,a.window.x0+a.window.dx/4,a.window.y0+14,a.window.dx/2,16);
      img = new PImage(window.dx,window.dy);

      title = _title;

      levels = new Field(n,m);  // generate data to plot
      for (int i=0; i<n; i++){
        for (int j=0; j<m; j++){
          levels.a[i][j] = range.out(i/PApplet.parseFloat(n-1));
        }
      }
    }
    public void display( float minv, float maxv ){
      if(box) {
        noStroke(); fill(colorScale(0));
        rect(0,0,width,45);
      }

      super.display(levels);

      int x0 = window.x0, x1 = window.x0+window.dx;
      int y0 = window.y0, y1 = window.y0+window.dy;
      float low = range.outS, high = range.outE;
      Scale x = new Scale(low,high,x0,x1);

      textSize(15);
      fill((sequential |!dark )?0:360);
      textAlign(RIGHT,CENTER);
      text(title,x0,0.5f*(y0+y1));
      textAlign(CENTER,BASELINE);
      text(""+low,x0,y0);
      text(""+high,x1,y0);
      if(low<0&&high>0)text("0",x.out(0),y0);
      textAlign(CENTER,TOP);
      stroke(0);
      fill(colorScale(minv));
      text(nf(minv,ceil(log(abs(minv))/log(10)),2),x.out(minv),y1);
      fill(colorScale(maxv));
      text(nf(maxv,ceil(log(abs(maxv))/log(10)),2),x.out(maxv),y1);
    }
  }
  
      public void displayTime(float t){
      int x0 = window.x0, x1 = window.x0+window.dx;
      int y0 = window.y0, y1 = window.y0+window.dy;
      textAlign(CENTER,BASELINE);
      fill(255);
      text("t = "+ nfs(t,2,2),0.5f*(x0+x1),y1);
    }
}
/**********************************
FreeInterface class

Extends SharpField by adding the density
  ratio (gamma) to compute the density 
  (rho)

Example code:

void setup(){
  size(400,400);

  FreeInterface f;
  int m=100,n=100;

  f = new FreeInterface(n,m,0.1);
  for( int i=0; i<n; i++){
    for( int j=0; j<m/3; j++){
      f.a[i][j] = 0.0;
    }
  }
  for( int i=n/2; i<n; i++){
    for( int j=m/3; j<2*m/3; j++){
      f.a[i][j] = 0.0;
    }
  }
  f.display(0,1);
  f.rho().display(1,2);
} 
***************************************/

class FreeInterface extends SharpField{
  float gamma;

  FreeInterface( int n, int m, float gamma){
    super( n, m, 0, 1 );
    this.gamma = gamma;
  }
  
  public VectorField rho(){
    VectorField b = new VectorField(n,m,1,1);
    for( int j=0; j<m; j++ ){
      for( int i=1; i<n; i++ ){
        b.x.a[i][j] = 0.5f*(a[i][j]+a[i-1][j])*(1.f-gamma)+gamma;
      }
      b.x.a[0][j] = b.x.a[1][j];
    }
    for( int i=0; i<n; i++ ){
      for( int j=1; j<m; j++ ){
        b.y.a[i][j] = 0.5f*(a[i][j]+a[i][j-1])*(1.f-gamma)+gamma;
      }
      b.y.a[i][0] = b.y.a[i][1];
    }
    return b;
  }
}
/*************************
 Inline Flapping Foil Test Class
 
 Example code:
 
InlineFoilTest test;
SaveData dat;
float maxT;
String datapath = "saved/";

int Re = 6000, nflaps = 2;
float stru = .45, stk = -135*PI/180, hc = 1, dAoA = 25*PI/180, uAoA = 0;
int resolution = 32, xLengths=7, yLengths=7, xStart = 3, zoom = 3;

void settings(){
  size(zoom*xLengths*resolution, zoom*yLengths*resolution);  
}
void setup() {
  maxT = (int)(2*hc/stru*resolution*nflaps);

  test = new InlineFoilTest(resolution, xLengths, yLengths, xStart, zoom, Re, true);
  test.setFlapParams(stru, stk, dAoA, uAoA, hc, "Sine");

  dat = new SaveData(datapath+"pressure.txt", test.foil.coords, resolution, xLengths, yLengths, zoom);
}
void draw() {
  test.update();
  test.display();
  dat.addData(test.t, test.foil.pressForce(test.flow.p), test.foil, test.flow.p);

  if (test.t>=maxT) {
    dat.finish();
    exit();
  }
}
void keyPressed() {
  dat.finish();
  exit();
}
***********************/

class InlineFoilTest {
  final int n, m;
  float dt = 0, t = 0, heaveAmp, dAoA, uAoA, inlineAmp, omega, chord = 1.0f, period, dfrac;
  float veloy, velox, adv_angle, recovery_angle, AoF, dAoFdt, v2, pitch=0, p=0, dpdt=0, integerr=0, yold=0;
  float Fd=0, Fd_dot=0, F, Fold;
  //float tau = .1, a=3, b=25; //note b/resolution is true time constant...
  float tau = .1f, a=3, b=50;
  int resolution;
  String pitchmethod;
  String filepath;

  boolean upstroke = false;

  NACA foil; 
  BDIM flow; 
  FloodPlot flood, flood2; 
  Window window;
  ReadData reader;

  InlineFoilTest( int resolution, int xLengths, int yLengths, int xStart, float zoom, int Re, boolean QUICK) {
    this.resolution = resolution;
    n = xLengths*resolution;
    m = yLengths*resolution;
    window = new Window(n, m);

    foil = new NACA(xStart*resolution, m/2, resolution*chord, .15f, window);
    foil.rotate(-foil.phi+PI);
    foil.rotate(0);
    
    flow = new BDIM(n, m, dt, foil, (float)resolution/Re, QUICK, -1);

    flood = new FloodPlot(window);
    flood.range = new Scale(-1, 1);
    flood.setLegend("vorticity");
    flood.setColorMode(1); 
    foil.setColor(0xffCCCCCC);

    flood2 = new FloodPlot(window);
    flood.range = new Scale(-0.5f, 0.5f);
    flood2.setLegend("pressure");
  }
  public void setFlapParams(float stru, float stk, float dAoA, float uAoA, float hc, String pitchmethod) {
    this.dAoA = dAoA; 
    this.uAoA = uAoA; 
    this.heaveAmp = hc*chord*resolution;
    this.inlineAmp = hc*chord*resolution*1/tan(stk);
    this.omega = TWO_PI/resolution * stru/(2*hc*chord);
    this.period = TWO_PI/omega;
    this.dfrac = 0.5f; //Functionality for changing the downstroke to period fraction not fully implemented
    foil.translate(this.inlineAmp, -this.heaveAmp);
    foil.translate(0,0);

    tau = tau*((float)resolution);
    this.pitchmethod = pitchmethod;

    adv_angle = atan2(-heaveAmp*omega, 1.f-inlineAmp*omega);
    recovery_angle = atan2(heaveAmp*omega, 1.f+inlineAmp*omega);
    
    if (pitchmethod.equals("FileRead")){
      readData();
    }
  }

  public void setControlParams(float a, float b, float tau) {
    this.a = a; 
    this.b = b;
    this.tau = tau*((float)resolution);
  }
  public void setFileRead(String filepath) {
    this.filepath = filepath;
  }
  public void readData(){
    reader = new ReadData(filepath);
  }
  public float controller(float y, float yd) {
    //Really bad observer
    y = y*.3f+yold*.7f; 
    yold = y;

    //Controller, in discrete
    float pnew = (yd/a+p/dt*b/a+AoF+1/(tau*a)*(integerr+.5f*dt*(yd-y)))/(1+b/a*1/dt);
    integerr = integerr+dt*(yd-y);
    p = pnew;

    println("AoA "+ (p-AoF)*180/PI);
    println("Phi "+ p*180/PI);
    println("dt "+ dt);
    return p;
  }

  public void computeState(float t) {
    computeVelocity(t);
    AoF = atan2(veloy, 1.f+velox);
    
    //float  = resolution*0.5;
    //AoF = atan2((heaveAmp*cos(omega*t)-heaveAmp*cos(omega*(t-delt)))/delt,(delt+inlineAmp*cos(omega*t)-inlineAmp*cos(omega*(t-delt)))/delt);
    
    v2 = (1+velox)*(1+velox)+veloy*veloy;

    PVector pforce = foil.pressForce(flow.p);
    //float ysign = ((pforce.y>0)?1:0)*2-1;
    //F = pforce.y*cos(foil.phi)-pforce.x*sin(foil.phi);
    F = pforce.y*cos(AoF)+pforce.x*sin(AoF);

//    t = t-dt/2.0;
//    dAoFdt = -heaveAmp*omega*omega*cos(omega*t)/(omega*omega*sin(omega*t)*sin(omega*t)*inlineAmp*inlineAmp-2*omega*sin(omega*t)*inlineAmp+heaveAmp*heaveAmp*omega*omega*sin(omega*t)*sin(omega*t)+1);
//    t = t+dt/2.0;
  }

  public void computeVelocity(float t) {
    float tnew = 0;
    if ((t%period)<(dfrac*period)) {
      tnew = t%period;
      veloy = -heaveAmp*omega/(dfrac*2)*sin(omega/(dfrac*2)*tnew);
      //velox = -inlineAmp*omega/(dfrac*2)*sin(omega/(dfrac*2)*tnew);
      velox = 0;
      upstroke = false;
    }
    else {
      tnew = (t%period)-period*dfrac;
      veloy = heaveAmp*omega/((1-dfrac)*2)*sin(omega/((1-dfrac)*2)*tnew);
      //velox = inlineAmp*omega/((1-dfrac)*2)*sin(omega/((1-dfrac)*2)*tnew);
      velox = 0;
      upstroke = true;
    }
  }

  public float computePitch(float t) {
    if (pitchmethod.equals("Cosine")){
      float pitchAmp = dAoA;
      if (upstroke) {
        pitchAmp = uAoA;
      }
      return pitchAmp/2-pitchAmp/2*cos(2*omega*t)+AoF;
    }

    if (pitchmethod.equals("Sine")){
      float pitchAmp = dAoA;
      if (upstroke) {
        pitchAmp = uAoA;
      }
      return pitchAmp*sin(omega*t)+AoF;
    }
    
    if (pitchmethod.equals("OpenLoop")){
      //Example: float stru = .5, stk = -120*PI/180, hc = 1.5, dAoA = 20*PI/180, uAoA = 20*PI/180, dfrac = .5;
      float pitchAmp = dAoA;
      if (upstroke) {
        pitchAmp = 0;
      }
      float vortcan = uAoA;
      return pitchAmp/2-pitchAmp/2*cos(2*omega*t)+vortcan/v2*cos(omega*t-1*PI/4.0f)+AoF;
    }
    
    if (pitchmethod.equals("OpenLoop2")){
      //Example: float stru = .5, stk = -120*PI/180, hc = 1.5, dAoA = 20*PI/180, uAoA = 20*PI/180, dfrac = .5;
      float pitchAmp = dAoA;
      if (upstroke) {
        pitchAmp = 0;
      }
      float vortcan = uAoA;
      return pitchAmp/2-pitchAmp/2*cos(2*omega*t)-vortcan/v2*dAoFdt+AoF;
    }
    if (pitchmethod.equals("OpenLoop3")){
      //Example: float stru = .5, stk = -120*PI/180, hc = 1.5, dAoA = 20*PI/180, uAoA = 20*PI/180, dfrac = .5;
      float pitchAmp = dAoA;
      if (upstroke) {
        pitchAmp = -uAoA;
      }
      Fd = a*v2*(pitchAmp/2-pitchAmp/2*cos(2*omega/(2*dfrac)*t));
      return (Fd/v2+a*AoF+b*foil.phi/dt)/(a+b/dt);
    }
    
    if (pitchmethod.equals("ClosedLoop")){
      float pitchAmp = dAoA;
      if (upstroke) {
        pitchAmp = -uAoA;
      }
      Fd = a*v2*(pitchAmp/2-pitchAmp/2*cos(2*omega/(2*dfrac)*t));
      return controller(F/v2, Fd/v2);
    }
    if (pitchmethod.equals("ClosedLoop2")){
      float pitchAmp = dAoA;
      if (upstroke) {
        pitchAmp = -uAoA;
      }
      Fd = a*v2*(pitchAmp/2-pitchAmp/2*cos(2*omega/(2*dfrac)*t));
      return controller(F/v2, Fd/v2);
    }
    
    if (pitchmethod.equals("Timeshift")){
      float phase = uAoA;
      uAoA = 0;
      pitchmethod = "Cosine";
      computeState(t-phase/omega);
      p = computePitch(t-phase/omega);
      pitchmethod = "Timeshift";
      uAoA = phase;
      computeState(t);
      return p;
    }
    if (pitchmethod.equals("NoTrail")){
      float dpdt = 4/3*(-veloy*cos(foil.phi)-(1-velox)*sin(foil.phi));
      return foil.phi+dpdt*dt;
    }

    if (pitchmethod.equals("Hummingbird")){
      //Taken from Tobalske profile: stru = .2, stk = -60*PI/180, hc = 1.5,
      return -25*PI/180*sin(omega*t)+(-5*PI/180*cos(omega*t)+5*PI/180)+10*PI/180;
    }
    
    if (pitchmethod.equals("FileRead")){
      int column = 0;
      p = reader.interpolate(t*omega % TWO_PI,column);
      println("Theta " + p);
      return p;
    }
    
    if (pitchmethod.equals("Turtle")){
      //Taken from Davenport profile: stru = .5, stk = -120*PI/180, hc = 1.5,
      return -90*PI/180*sin(omega*t)*(0.5f*cos(omega*t)+0.5f)-10*PI/180;
    }
    
    if (pitchmethod.equals("NoPitch")){
      return 0;
    }
    if (pitchmethod.equals("SinusoidPitch")){
      return dAoA*sin(omega*t);
    }
    
    if (pitchmethod.equals("Feather")){
      return AoF;
    }
    throw new Error("Not a valid pitch method - check your spelling perhaps?");
  }
  public void update() {
    if (flow.QUICK) {
      dt = flow.checkCFL();
      flow.dt = dt;
    }

    computeState(t);
    pitch = computePitch(t);
    foil.rotate(-foil.phi-pitch+PI);
    println("AoA: "+(pitch-AoF)*180/PI);

    computeVelocity(t-dt/2.0f); //Recompute velocity for mid-timestep, fixes Euler drift
    foil.translate(velox*dt, -veloy*dt);

    flow.update(foil);flow.update2();
    t += dt;
  }
  public void display() {
    flood.display(flow.u.curl());
    foil.display();
    foil.displayVector(foil.pressForce(flow.p));
  }
}
/*******************************************
 Emplements a 1D body defined by an array of points
 The body is given a thickness of 2 cells.
 
 example code:
BDIM flow;
Body line; 
void setup() {
  int n=(int)pow(2, 7);
  size(400, 400);      
  Window view = new Window(n, n);
  line = new LineSegBody(n/3., n/2., view);
  for ( int i=0; i<n/3.; i++ ) {
    line.add(n/3.+i, n/2.+5.*sin(TWO_PI*i/(n/3.)));
  }
  line.end();
  flow = new BDIM(n, n, 0.5, line,0.01,true);  
}

void draw() {
  flow.update();
  flow.update2();
  flow.u.curl().display(-0.5,0.5);
  line.display();
}
********************************/

class LineSegBody extends Body {
  float thk=2, weight;

  LineSegBody( float x, float y, Window window ) {
    super(x, y, window); 
    weight = window.pdx(thk);
  }

  public void add( float x, float y ) {
    coords.add( new PVector( x, y ) );
  }

  public void end() {
    super.end(false);
  }

  public void display( int C, Window window ) { // note: can display while adding
    stroke(C); 
    noFill(); 
    strokeWeight(weight);
    beginShape();
    for ( PVector x : coords ) vertex(window.px(x.x), window.py(x.y));
    endShape();
  }

  public float distance( float x, float y ) { // in cells
    float dis;
    if (n>4) { // distance to bounding box
      dis = box.distance(x, y);
      if (dis>3+thk) return dis;
    }
    // get dist to each line segment, choose min
    dis = 1e10f;
    for ( OrthoNormal o : orth ) dis = min(dis, o.distance(x, y, false));
    return dis-0.5f*thk;
  }

  public PVector pressForce ( Field p ) {
    PVector pv = new PVector(0, 0);
    for ( int s=-1; s<=1; s+=2 ) {
      for ( OrthoNormal o : orth ) {
        float pdl = p.linear( o.cen.x+0.5f*s*thk*o.nx, o.cen.y+0.5f*s*thk*o.ny )*o.l;
        pv.add(s*pdl*o.nx, s*pdl*o.ny, 0);
      }
    }
    return pv;
  }
}
/**********************************
MG (MultiGrid) class

Sets up a PossionMatrix problem and solves with MG

Example code:

MG solver;
void setup(){
  size(512,512);
  noStroke();
  frameRate(.5);
  int N = 128+2;                               // <-Try changing the number of points
  VectorField u = new VectorField(N,N,1,-0.5);
  u.x.eq(0.,40,50,40,75);                      // Create divergent velocity field
  VectorField c = new VectorField(N,N,0,0);
  c.x.eq(1); c.y.eq(1); c.setBC();
  solver = new MG( new PoissonMatrix(c), new Field(N,N), u.divergence() );
  println(solver.tol);                          //  <- note dependance on N
}
void draw(){
  solver.update();                             // Do one solver iteration
  solver.r.display(-1e-5,1e-5);                // Visualize the residual
  println("ratio:"+solver.r.inner(solver.r)/solver.tol+", inf:"+solver.r.L_inf());
  if(solver.r.L_inf()<1e-5) noLoop();
}

 ***********************************/

public Field MGsolver( float itmx, PoissonMatrix A, Field x, Field b ) {
  MG solver = new MG( A, x, b);
  while (solver.iter<itmx) {
    solver.update();
    if (solver.r.inner(solver.r)<solver.tol) break;
  }
  //println("residual: "+solver.r.L_inf()+", iter: "+solver.iter);
  return solver.x;
}

class MG {
  PoissonMatrix A;
  Field r, x, d;
  int iter=0, level=0, its=4;
  float tol=1e-4f;

  MG( PoissonMatrix A, Field x, Field b ) {
    this.A = A;
    this.x = x;
    r = new Field(x.n, x.m, 0, tol);
    tol = r.inner(r);
    r = A.residual(b, x);
  }
  MG( PoissonMatrix A, Field r, int level, int iter ) {
    this.A = A;
    this.r = r;
    x = new Field(r.n, r.m);
    this.level = level;
    this.iter = iter;
  }

  public void update() {
    iter++;
    //println("  iter:"+iter);
    vCycle();
    smooth(its);
  }

  public void vCycle() { // recursive MG V-cycle
    smooth(0);
    MG coarse = new MG(restrict(A), restrict(r), level+1, iter);
    //println("  level:"+level+"->"+coarse.level);
    if (coarse.divisible()) coarse.vCycle();
    coarse.smooth(its);
    //println("  level:"+coarse.level+"->"+level);
    d = prolongate(coarse.x);
    increment();
  }

  public void smooth( int itmx ){
    d = r.times(A.inv);
    for( int it=0 ; it<itmx ; it++ ){
    for( int i=1 ; i<r.n-1 ; i++ )  {
    for( int j=1 ; j<r.m-1 ; j++ )  {
      d.a[i][j] = -(d.a[i-1][j]*A.lower.x.a[i][j]
                   +d.a[i+1][j]*A.lower.x.a[i+1][j]
                   +d.a[i][j-1]*A.lower.y.a[i][j]
                   +d.a[i][j+1]*A.lower.y.a[i][j+1]
                   -r.a[i][j])*A.inv.a[i][j];
    }}}
    d.setBC();
    increment();
  }

  public void increment() {
    x.plusEq(d);
    r.minusEq(A.times(d));
  }

  public boolean divisible() {
    boolean flag = (x.n-2)%2==0 && (x.m-2)%2==0 && x.n>4 && x.m>4;
    if ( !flag && x.n>9 && x.m>9 ) {
      println("MultiGrid requires the size in each direction be a large factor of two (2^p) times a small number (N=1..9).");
      exit();
    }
    return flag;
  }

  public PoissonMatrix restrict( PoissonMatrix A ) {
    int n = (A.lower.n-2)/2+2;
    int m = (A.lower.m-2)/2+2;
    VectorField lower = new VectorField(n, m, 0, 0);
    for ( int i=1; i<n-1; i++ ) {
      for ( int j=1; j<m-1; j++ ) {
        int ii = (i-1)*2+1;
        int jj = (j-1)*2+1;
        lower.x.a[i][j] = (A.lower.x.a[ii][jj]+A.lower.x.a[ii][jj+1])*0.5f;
        lower.y.a[i][j] = (A.lower.y.a[ii][jj]+A.lower.y.a[ii+1][jj])*0.5f;
      }
    }
    lower.setBC();
    return new PoissonMatrix(lower);
  }

  public Field restrict( Field a ) {
    int n = (a.n-2)/2+2;
    int m = (a.m-2)/2+2;
    Field b = new Field(n, m);
    for ( int i=1; i<n-1; i++ ) {
      for ( int j=1; j<m-1; j++ ) {
        int ii = (i-1)*2+1;
        int jj = (j-1)*2+1;
        b.a[i][j] = a.a[ii][jj]+a.a[ii][jj+1]+a.a[ii+1][jj]+a.a[ii+1][jj+1];
      }
    }
    b.setBC();
    return b;
  }

  public Field prolongate( Field a ) {
    int n = (a.n-2)*2+2;
    int m = (a.m-2)*2+2;
    Field b = new Field(n, m);
    for ( int i=0; i<n; i++ ) {
      for ( int j=0; j<m; j++ ) {
        int ii = (i-1)/2+1;
        int jj = (j-1)/2+1;
        b.a[i][j] = a.a[ii][jj];
      }
    }
    b.setBC();
    return b;
  }
}
/********************************
NACA airfoil class

example code:

NACA foil;
void setup(){
  size(400,400);
  foil = new NACA(1,1,0.5,0.20,new Window(3,3));
}
void draw(){
  background(0);
  foil.display();
  foil.rotate(0.01);
}
********************************/
class NACA extends Body{
  int m = 100;
  float c, FoilArea;
  float pivot;
  
  NACA( float x, float y, float c, float t, float pivot, Window window ){
    super(x,y,window);
    add(xc.x-c*pivot,xc.y);
    //print("This is "+(xc.x-c*pivot)+xc.x+(xc.x+c*(1-pivot)));
    for( int i=1; i<m; i++ ){
      float xx = pow(i/(float)m,2);
      add(xc.x+c*(xx-pivot),xc.y+t*c*offset(xx));      
    }
    add(xc.x+c*(1-pivot),xc.y);
    for( int i=m-1; i>0; i-- ){
      float xx = pow(i/(float)m,2);
      add(xc.x+c*(xx-pivot),xc.y-t*c*offset(xx));
    }
    end(); // finalizes shape
    this.c = c;
    FoilArea = t*c*0.685084f;    //crossectional area of NACA foil
    
    float dx = c/2, dy = t*c/2;
    ma = new PVector(PI*sq(dy),PI*sq(dx),0.125f*PI*sq(sq(dx)-sq(dy)));
    ma.z += sq(c*(0.5f-pivot))*ma.y;
  }
  
  NACA( float x, float y, float c, float t, Window window ){
    this(x,y,c,t,.25f,window);
  }
  
  public float[][] interp( Field a ){
    float[][] b = new float[2][m+1];

    PVector x = coords.get(0);
    b[0][0] = a.interp(x.x,x.y); b[1][0] = b[0][0];
    for ( int i = 1; i<m; i++ ){
      x = coords.get(i);
      b[0][i] = a.interp(x.x,x.y);
      x = coords.get(n-i);
      b[1][i] = a.interp(x.x,x.y);
    }
    x = coords.get(m);
    b[0][m] = a.interp(x.x,x.y); b[1][m] = b[0][m];
    return b;
  }

  public float offset( float x ){
    return 5*(0.2969f*sqrt(x)-0.1260f*x-0.3516f*pow(x,2)+0.2843f*pow(x,3)-0.1015f*pow(x,4));
  }
}
public float invSqrt(float x) {//fast inv sqrt; f=1/x^(1/2)
  float xhalf = 0.5f * x;
  int i = Float.floatToIntBits(x);
  i = 0x5f3759df - (i >> 1);
  x = Float.intBitsToFloat(i);
  x *= (1.5f - xhalf * x * x);
  x *= (1.5f - xhalf * x * x);//can be deleted for quicker calculation
  return x;
}


public float P_x(Body body, Field p, int i){ 
  return (body.orth[i].ty)*p.a[(int)body.coords.get(i).x][(int)body.coords.get(i).y];
}
public float Fx_Sum(Body body, Field p){
  float Psum = 0;
  for (int i=0; i<body.n; i++){
    Psum += P_x(body, p, i)*body.orth[i].l;
    //println("Px= "+P_x(body, p, i)+" Py= "+body.orth[i].l);
  }
  return Psum;
}
public float P_y(Body body, Field p, int i){ 
  return (body.orth[i].tx)*p.a[(int)body.coords.get(i).x][(int)body.coords.get(i).y];
}
public float Fy_Sum(Body body, Field p){
  float Psum = 0;
  for (int i=0; i<body.n; i++){
    Psum += P_y(body, p, i)*body.orth[i].l;
    //println("Px= "+P_x(body, p, i)+" Py= "+body.orth[i].l);
  }
  return Psum;
}
public float M(Body body,Field p, int i){
  float M_x = P_x(body,p,i)*body.orth[i].l*(body.coords.get(i).y-body.coords.get(0).y);
  float M_y = P_x(body,p,i)*body.orth[i].l*(body.coords.get(i).x-body.coords.get(0).x);
  return M_x+M_y;
}
public float M_Sum(Body body, Field p){
  float Msum = 0;
  for (int i=0; i<body.n; i++){
    Msum += M(body, p, i);
    //println("Px= "+P_x(body, p, i)+" Py= "+body.orth[i].l);
  }
  return Msum;
}
public float P_input(float M_Sum,int time,float Max_theta, float Rotate_omega){
  return -M_Sum*Max_theta*Rotate_omega*cos(Rotate_omega*time);
}
public float eff(float Psum, float Msum){
  return Psum/Msum;
}




// Class to hold the ortho-normal project terms of a line segment

class OrthoNormal{
  float l,nx,ny,tx,ty,off,t1,t2;
  PVector cen;
  //PVector l1;//invSqrt

  OrthoNormal(){ this(new PVector(0,0), new PVector(0,1));}
  OrthoNormal(PVector x1, PVector x2 ){ // set the ortho-normal values based on two points
    l = PVector.sub(x1,x2).mag();
    // tx = (x2.x-x1.x)/l;    // x tangent
    // ty = (x2.y-x1.y)/l;    // y tangent
    //l1 = PVector.sub(x1,x2);
    tx = (x2.x-x1.x)*invSqrt((x2.x-x1.x)*(x2.x-x1.x)+(x2.y-x1.y)*(x2.y-x1.y));//invSqrt
    ty = (x2.y-x1.y)*invSqrt((x2.x-x1.x)*(x2.x-x1.x)+(x2.y-x1.y)*(x2.y-x1.y));
    t1 = x1.x*tx+x1.y*ty;  // tangent location of point 1
    t2 = x2.x*tx+x2.y*ty;  // tangent location of point 2
    nx = -ty; ny = tx;     // normal vector
    off = x1.x*nx+x1.y*ny; // normal offset
    cen = PVector.add(x1,x2); // centriod
    cen.div(2.f);
  }

  public float distance( float x, float y, Boolean projected){ // distance function
    float d = x*nx+y*ny-off; // normal distance to line 
    if(projected) return d;  // |distance|_n (signed, fastest)
    float d1 = x*tx+y*ty-t1; // tangent dis to start
    float d2 = x*tx+y*ty-t2; // tangent dis to end
//    return sqrt(sq(d)+sq(max(0,-d1))+sq(max(0,d2))); // |distance|_2
    return abs(d)+max(0,-d1)+max(0,d2);              // |distance|_1 (faster)
  }
  public float distance( float x, float y){ return distance(x,y,true); } // default to signed dist

  public float tanCoord( float x, float y ){ // tangent coordinate
    return min(max((x*tx+y*ty-t1)/l,0),1);
  }

  public void translate(float dx, float dy){
    t1 += dx*tx+dy*ty;
    t2 += dx*tx+dy*ty;
    off += dx*nx+dy*ny;
    cen.add(dx,dy,0);
  }

  public void print(){
    println("t=["+tx+","+ty+"]");
    println("n=["+nx+","+ny+"]");
    println("offsets=["+t1+","+t2+","+off+"]");
  }
}
/********************************
NACA airfoil class

example code:

NACA foil;
void setup(){
  size(400,400);
  foil = new NACA(1,1,0.5,0.20,new Window(3,3));
}
void draw(){
  background(0);
  foil.display();
  foil.rotate(0.01);
}
********************************/
class Part extends Body{
  int m = 100;
  float c, FoilArea;
  float pivot;
  
  Part( float x, float y, float c, float t, float pivot, Window window ){
    super(x,y,window);
    add(xc.x-c*pivot,xc.y);
    //print("This is "+(xc.x-c*pivot)+xc.x+(xc.x+c*(1-pivot)));
    for( int i=1; i<m; i++ ){
      float xx = i/(float)m;
      add(xc.x+c*(xx-pivot),xc.y+c*offset(xx,t)/2);      
    }
    add(xc.x+c*(1-pivot),xc.y);
    for( int i=m-1; i>0; i-- ){
      float xx = i/(float)m;
      add(xc.x+c*(xx-pivot),xc.y-c*offset(xx,t)/2);
    }
    end(); // finalizes shape
    this.c = c;
    //FoilArea = t*c*0.685084;    //crossectional area of Part foil
    FoilArea = t*c*0.985084f;
    float dx = c/2, dy = t*c/2;
    ma = new PVector(PI*sq(dy),PI*sq(dx),0.125f*PI*sq(sq(dx)-sq(dy)));
    ma.z += sq(c*(0.5f-pivot))*ma.y;
  }
  
  Part( float x, float y, float c, float t, Window window ){
    this(x,y,c,t,.25f,window);
  }
  
  public float[][] interp( Field a ){
    float[][] b = new float[2][m+1];

    PVector x = coords.get(0);
    b[0][0] = a.interp(x.x,x.y); b[1][0] = b[0][0];
    for ( int i = 1; i<m; i++ ){
      x = coords.get(i);
      b[0][i] = a.interp(x.x,x.y);
      x = coords.get(n-i);
      b[1][i] = a.interp(x.x,x.y);
    }
    x = coords.get(m);
    b[0][m] = a.interp(x.x,x.y); b[1][m] = b[0][m];
    return b;
  }

  public float offset( float x, float t ){
    if(x<t){
      return x;
    }
    else if (x>(1-t)) {
      return (1-x);
    }
    else return t;
    //return 5*(0.2969*sqrt(x)-0.1260*x-0.3516*pow(x,2)+0.2843*pow(x,3)-0.1015*pow(x,4));
  }
}
/********************************
NACA airfoil class

example code:

NACA foil;
void setup(){
  size(400,400);
  foil = new NACA(1,1,0.5,0.20,new Window(3,3));
}
void draw(){
  background(0);
  foil.display();
  foil.rotate(0.01);
}
********************************/
class Part_head extends Body{
  int m = 100;
  float c, FoilArea;
  float pivot;
  
  Part_head( float x, float y, float c, float t, float pivot, Window window ){
    super(x,y,window);
    add(xc.x-c*pivot,xc.y);
    //print("This is "+(xc.x-c*pivot)+xc.x+(xc.x+c*(1-pivot)));
    for( int i=1; i<m; i++ ){
      float xx = i/(float)m;
      add(xc.x+c*(xx-pivot),xc.y+c*offset(xx,t,pivot)/2);      
    }
    add(xc.x+c*(1-pivot),xc.y);
    for( int i=m-1; i>0; i-- ){
      float xx = i/(float)m;
      add(xc.x+c*(xx-pivot),xc.y-c*offset(xx,t,pivot)/2);
    }
    end(); // finalizes shape
    this.c = c;
    this.pivot = pivot;
    //FoilArea = t*c*0.685084;    //crossectional area of Part_head foil
    FoilArea = t*c*0.985084f;
    float dx = c/2, dy = t*c/2;
    ma = new PVector(PI*sq(dy),PI*sq(dx),0.125f*PI*sq(sq(dx)-sq(dy)));
    ma.z += sq(c*(0.5f-pivot))*ma.y;
  }
  
  Part_head( float x, float y, float c, float t, Window window ){
    this(x,y,c,t,.25f,window);
  }
  
  public float[][] interp( Field a ){
    float[][] b = new float[2][m+1];

    PVector x = coords.get(0);
    b[0][0] = a.interp(x.x,x.y); b[1][0] = b[0][0];
    for ( int i = 1; i<m; i++ ){
      x = coords.get(i);
      b[0][i] = a.interp(x.x,x.y);
      x = coords.get(n-i);
      b[1][i] = a.interp(x.x,x.y);
    }
    x = coords.get(m);
    b[0][m] = a.interp(x.x,x.y); b[1][m] = b[0][m];
    return b;
  }
  //float head_end = pivot;
  public float offset( float x, float t, float pivot){
    float ym=5*(0.2969f*sqrt(pivot)-0.1260f*pivot-0.3516f*pow(pivot,2)+0.2843f*pow(pivot,3)-0.1015f*pow(pivot,4));
    if(x<pivot){
      return 5*(0.2969f*sqrt(x)-0.1260f*x-0.3516f*pow(x,2)+0.2843f*pow(x,3)-0.1015f*pow(x,4))*(t/ym);
    }
    else if (x>(1-t)) {
      return (1-x);
    }
    else return t;
    //return 5*(0.2969*sqrt(x)-0.1260*x-0.3516*pow(x,2)+0.2843*pow(x,3)-0.1015*pow(x,4));
  }
}
/********************************
NACA airfoil class

example code:

NACA foil;
void setup(){
  size(400,400);
  foil = new NACA(1,1,0.5,0.20,new Window(3,3));
}
void draw(){
  background(0);
  foil.display();
  foil.rotate(0.01);
}
********************************/
class Part_tail extends Body{
  int m = 100;
  float c, FoilArea;
  float pivot;
  
  Part_tail( float x, float y, float c, float t, float pivot, Window window ){
    super(x,y,window);
    add(xc.x-c*pivot,xc.y);
    //print("This is "+(xc.x-c*pivot)+xc.x+(xc.x+c*(1-pivot)));
    for( int i=1; i<m; i++ ){
      float xx = i/(float)m;
      add(xc.x+c*(xx-pivot),xc.y+c*offset(xx,t,pivot)/2);      
    }
    add(xc.x+c*(1-pivot),xc.y);
    for( int i=m-1; i>0; i-- ){
      float xx = i/(float)m;
      add(xc.x+c*(xx-pivot),xc.y-c*offset(xx,t,pivot)/2);
    }
    end(); // finalizes shape
    this.c = c;
    this.pivot = pivot;
    //FoilArea = t*c*0.685084;    //crossectional area of Part_tail foil
    FoilArea = t*c*0.985084f;
    float dx = c/2, dy = t*c/2;
    ma = new PVector(PI*sq(dy),PI*sq(dx),0.125f*PI*sq(sq(dx)-sq(dy)));
    ma.z += sq(c*(0.5f-pivot))*ma.y;
  }
  
  Part_tail( float x, float y, float c, float t, Window window ){
    this(x,y,c,t,.25f,window);
  }
  
  public float[][] interp( Field a ){
    float[][] b = new float[2][m+1];

    PVector x = coords.get(0);
    b[0][0] = a.interp(x.x,x.y); b[1][0] = b[0][0];
    for ( int i = 1; i<m; i++ ){
      x = coords.get(i);
      b[0][i] = a.interp(x.x,x.y);
      x = coords.get(n-i);
      b[1][i] = a.interp(x.x,x.y);
    }
    x = coords.get(m);
    b[0][m] = a.interp(x.x,x.y); b[1][m] = b[0][m];
    return b;
  }

  public float offset( float x, float t, float pivot){
    float ym=5*(0.2969f*sqrt(pivot)-0.1260f*pivot-0.3516f*pow(pivot,2)+0.2843f*pow(pivot,3)-0.1015f*pow(pivot,4));
    if(x<t){
      return x;
    }
    else if (x>pivot) {
      return 5*(0.2969f*sqrt(x)-0.1260f*x-0.3516f*pow(x,2)+0.2843f*pow(x,3)-0.1015f*pow(x,4))*(t/ym);
    }
    else return t;
    //return 5*(0.2969*sqrt(x)-0.1260*x-0.3516*pow(x,2)+0.2843*pow(x,3)-0.1015*pow(x,4));
  }
  // float offset( float x, float t, float pivot){
  //   float ym=5*(0.2969*sqrt(pivot)-0.1260*pivot-0.3516*pow(pivot,2)+0.2843*pow(pivot,3)-0.1015*pow(pivot,4));
  //   if(x<pivot){
  //     return 5*(0.2969*sqrt(x)-0.1260*x-0.3516*pow(x,2)+0.2843*pow(x,3)-0.1015*pow(x,4))*(t/ym);
  //   }
  //   else if (x>(1-t)) {
  //     return (1-x);
  //   }
  //   else return t;
  //   //return 5*(0.2969*sqrt(x)-0.1260*x-0.3516*pow(x,2)+0.2843*pow(x,3)-0.1015*pow(x,4));
  // }
}
/*
Particle class

This class defines Lagrangian marker Particles which can be advected with the flow.

example code:

BDIM flow;
CircleBody body;
Particle marker;

void setup(){
  int n=(int)pow(2,6); size(400,400);
  Window view = new Window(n,n);
  body = new CircleBody(n/3,n/2,n/8,view);
  flow = new BDIM(n,n,1,body);
  marker = new Particle( 0, 32, color(0), view, 300 );
}
void draw(){
  flow.update(); flow.update2();
  marker.update(flow.u,flow.u0,flow.dt);
  marker.display();
  body.display();
  if(marker.dead()) marker = new Particle( 0, 32, color(0), marker.window, 300 );
}
**************************/

class Particle {
  Window window;
  int bodyColor;
  PVector x,x0;
  int step=0, lifeSpan;
  
  Particle( float x0, float y0, int _color, Window _window, int _lifeSpan, int _step ) {
    x = new PVector( x0 , y0 );
    bodyColor = _color;
    window = _window;
    lifeSpan = _lifeSpan;
    step = _step;
  }
  Particle( float x0, float y0, int _color, Window _window, int _lifeSpan ) { this(x0,y0,_color,_window,_lifeSpan,0); }
  
  public void update( VectorField u, VectorField u0, float dt ){
    x0 = x.copy();
    float px = x.x+dt*u0.x.quadratic( x.x, x.y ); // forward
    float py = x.y+dt*u0.y.quadratic( x.x, x.y ); //   projection
    float p0x = px-dt*u.x.quadratic( px, py );    // backward 
    float p0y = py-dt*u.y.quadratic( px, py );    //   projection
    x.set(px+(x.x-p0x)*0.5f,py+(x.y-p0y)*0.5f,0);   // O(2) update
    step++;
  }

  public void display(){display(bodyColor);}
  public void display(int C){
    stroke(C);
    line(window.px(x0.x),window.py(x0.y),window.px(x.x),window.py(x.y));
  }
  
  public void displayPoint(){displayPoint(bodyColor);}
  public void displayPoint(int C){
    noStroke(); fill(C); ellipse(window.px(x.x),window.py(x.y),4,4);
  }

  public boolean dead(){
    return ( window.py(x.y)<-10 || window.px(x.x)>width+10 || window.py(x.y)>height+10 || step > lifeSpan );
  }
}
/******************************************
ParticlePlot

Draw tracer particles and color them in based on a scalar field

BDIM flow;
Body body;
ParticlePlot plot;

void setup(){
  int n = (int)pow(2,6); size(400,400);
  Window window = new Window(n,n);
  body = new CircleBody( n/3, n/2, n/8, window );
  body.bodyColor = color(#00008B);
  flow = new BDIM(n,n,1,body);

  plot = new ParticlePlot( window, 10000 );
  plot.setColorMode(4);
  plot.setLegend("Vorticity",-0.5,0.5);
}

void draw(){
  flow.update(body); flow.update2();
  plot.update(flow); // !NOTE!
  plot.display(flow.u.curl());
  body.display();
}
********************************************/

class ParticlePlot extends FloodPlot{
  FieldSwarm swarm;
  
  ParticlePlot( Window window, int num ){
    super( window );
    swarm = new FieldSwarm( window, num );
  }

  public void update( BDIM flow ){ swarm.update(flow); }
  public void update( VectorField u, VectorField u0 , float dt){ swarm.update( u, u0, dt ); }

  public void display ( Field a ){
    float minv = 1e6f, maxv = -1e6f;
    colorMode(HSB,360,1.0f,1.0f,1.0f);
    int c = sequential?colorScale(range.out(0.6f)):color(300); fill(c,0.2f);
    noStroke(); rect(0,0,width,height); // Draw partially transparent background to get streaky effect
    strokeWeight(2);
    for( Particle p: swarm.pSet){
      float f = a.interp(p.x.x,p.x.y);
      p.display(colorScale(f));
      minv = min(minv,f);
      maxv = max(maxv,f);
    }
    if(legendOn) {
      legend.box = true;
      legend.display(minv,maxv);
    }
  }
}
/**********************************
PoissonMatrix class

Holds values and operators for a Poisson matrix

   A*x = div{lower*grad{x}} = b      (1)

where lower is a VectorField which are the 
lower diagonal components of the matrix A.
From Eq.1 the matrix is symmetric and the
diagonals are given by 

  sum{a_{i,j},i} = 0 forall j       (2)

example code:

void setup(){
  colorMode(RGB,1.0);
  size(500,500);
  VectorField c = new VectorField(40,40,0,0);
  c.x.eq(1.); c.y.eq(1.); c.setBC();
  PoissonMatrix A = new PoissonMatrix(c);
  Field x = new Field(40,40,0,1);
  for(int i=0; i<40; i++ ){
  for(int j=0; j<40; j++ ){
    x.a[i][j] = i*i-j*j;
  }
  }
  Field Ax = A.times(x);
  Ax.display(-2,2);
}
***********************************/
public class PoissonMatrix {
  int n, m;
  VectorField lower;
  Field diagonal,inv;

  PoissonMatrix( VectorField lower ){
    this.n = lower.n;
    this.m = lower.m;
    this.lower = new VectorField(lower);
    diagonal = new Field(n,m);
    inv = new Field(n,m,0,1);
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      float sumd = lower.x.a[i][j]+lower.x.a[i+1][j]
                  +lower.y.a[i][j]+lower.y.a[i][j+1];
      diagonal.a[i][j] = -sumd;
      if(sumd>1e-5f) inv.a[i][j] = -1.f/sumd;
    }}
  }

  public Field times( Field x ){
    Field ab = new Field(n,m);
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      ab.a[i][j] = x.a[i][j]*diagonal.a[i][j]
                  +x.a[i-1][j]*lower.x.a[i][j]
                  +x.a[i+1][j]*lower.x.a[i+1][j]
                  +x.a[i][j-1]*lower.y.a[i][j]
                  +x.a[i][j+1]*lower.y.a[i][j+1];
    }} 
    return ab;
  }

  public Field residual( Field b, Field x ){
    return b.minus(this.times(x));
  }
}
/**********************************
 ReadData class
 
 Reads timestamped data from a file to use in Lilypad
 
 ----example code fragment:
 reader = new ReadData("FileToRead.txt");
 float dat = reader.interpolate(t, column);
 ----see use in InlineFoilTest.pde
 
 ----the file should look like this, with Tabs between the data:
 #rows #cols dt <AnyOtherInfoYouWant (Optional)>
 data00 data01 data02....
 data10 data11 data12....
 ...
 
 each row is the data at a timestep - 0zth row is t=0, 1st is t=dt, etc
 
 ----you can create this file type in matlab with these commands (if 'arr' is the array of data, 'dt' is your timestep, and 'filepath' is a string that points to a text file)
header = [size(arr,1) size(arr,2) dt <AnyOtherInfoYouWant>]
save(filepath,'header','-ascii', '-tabs');
save(filepath,'arr','-append','-ascii', '-tabs');
 ***********************************/

class ReadData {
  float[][] data;
  BufferedReader reader;
  String line;
  float dat=0, dt=1;
  String[] header;
  int rows, cols;
  boolean verbose = false;
  char delim = TAB;

  ReadData(String filepath) {
    reader = createReader(filepath);
    readAll();
  }

  public void readAll() {
    try {
      line = reader.readLine();
      header = split(line, delim);
      rows = PApplet.parseInt(PApplet.parseFloat(header[0]));
      cols = PApplet.parseInt(PApplet.parseFloat(header[1]));
      dt =  PApplet.parseFloat(header[2]);
      data = new float[rows][cols];
    }
    catch (Exception e) {
      println("Error - Bad Header");
      println(rows);
      e.printStackTrace();
    }
    if (verbose) {
      println("#Rows: " + rows + "  #Cols: " + cols + "  dt: " + dt);
    }
    for (int i=0; i<rows; i++) {
      try {
        line = reader.readLine();
      } 
      catch (Exception e) {
        println("Error - Ran out of Data");
        e.printStackTrace();
      }
      String[] pieces = split(line, delim);
      for (int j=0; j<cols; j++) {
        dat = PApplet.parseFloat(pieces[j]);
        if (verbose) {
          println("Row: " + i + "  Col: " + j + "  t: " + i*dt + "  Data: " + dat + "#Rows: " + rows);
        }
        data[i][j] = dat;
      }
    }
  }
  public float interpolate(float t, int column) {
    int index = floor(t/dt)%rows;
    int next = ceil(t/dt)%rows;
    if (verbose) {
      println("Index: " + index + "  Next: " + next + "  t: " + t + "  Column: " + column);
    }
    if (column>=cols || column<0) {
      throw new Error("Requested data column not within data");
    }

    if (index==next) {
      return data[index][column];
    }

    float slope = (data[next][column]-data[index][column])/dt;
    return data[index][column]+(t%(dt*rows)-index*dt)*slope;
  }
} 
/**********************************
 SaveData class
 
Saves data to a text file with customizable header
 
---- example code fragment:
 dat = new SaveData("pressure.txt",test.foil.fcoords,resolution,xLengths,yLengths,zoom);
 dat.addData(test.t, test.flow.p);
 dat.finish();
----see use in InlineFoilTest.pde, AudreyTest.pde, etc
***********************************/
 
class SaveData{
  Body body;
  ArrayList<PVector> coords;
  PrintWriter output;
  int n;
  
  SaveData(String name, ArrayList<PVector> coords, int resolution, int xLengths, int yLengths, int zoom){
    output = createWriter(name);
    this.coords = coords;
    n = coords.size();
    output.println("%% Pressure distribution along the foil using processing viscous simulation");
    output.print("% xcoord = [");
    for(int i=0; i<n; i++){
      output.print(coords.get(i).x +" ");
    }
    output.println("];");
    
    output.print("% ycoord = [");
    for(int i=0; i<n; i++){
      output.print(coords.get(i).y +" ");
    }
    output.println("];");
  
    output.print("% resolution = "+ resolution);
    output.print("; xLengths = "+ xLengths);
    output.print("; yLengths = "+ yLengths);
    output.print("; zoom = "+ zoom);

    output.println(";");  
  }
  SaveData(String name, Body body,int resolution, int xLengths, int yLengths, int zoom){
    this(name, body.coords, resolution, xLengths, yLengths, zoom);
    this.body=body;
  }
  SaveData(String name, ArrayList<PVector> coords){
    this(name, coords,-1,-1,-1,-1);
  }
  SaveData(String name, Body body){
    this(name, body.coords,-1,-1,-1,-1);
  }
     
  public void saveParam(String name, float value){
    output.println("% " + name + " = " + value + ";");
  }
  public void saveString(String s){
    output.println(s);
  }
  
  public void addData(float t, Field a){
    output.print(t + " ");
    for(int i=0; i<n; i++){
      output.print(a.linear( coords.get(i).x, coords.get(i).y ) +" ");
    }
    output.println(";");
  }
  
  public void addData(float t, PVector p, Body b, Field a){
    //output.print(t + " ");
    output.println(t + " <-time ");
    //output.print(p.x + " " + p.y + " ");
    output.println(p.x + " <-P(x) " + p.y + " <-P(y) ");
    output.println(b.xc.x + " " + b.xc.y + " ");
    
    for(int i=0; i<n; i++){
      output.print(b.coords.get(i).x + " " + b.coords.get(i).y + " <-coords ");
      output.println(a.linear( b.coords.get(i).x, b.coords.get(i).y ) +" <-pressure ");
      //output.print("-");
    }
    
    output.println(";");
    output.println("");
  }
  
  public void addDataSimple(float t, PVector p, Body b, Field a){  //simplified to only output time and vector x and y values 
    output.print(t + " ");
    output.print(p.x + " " + p.y + " ");
    output.println(";");
  }

  public void addText(String s){   //allows to insert text anywhere in txt
    output.println(s);
  }

  public void finish(){
    output.flush(); // Writes the remaining data to the file
    output.close(); // Finishes the file
  }
  public void addFxSum(float t, Body b, Field a,float Max_theta, float Rotate_omega){ 
    output.println(t + " <-time ");  
    output.println(Fx_Sum(b,a) + " <- F(x) " + "     ");
    output.println(Fy_Sum(b,a) + " <- F(y) " + "     ");
    output.println(M_Sum(b,a) + " <- M " + "     ");
    output.println(P_input(M_Sum(b,a),(int)t, Max_theta, Rotate_omega) + " <- P input " + "     ");
    output.println(eff(Fx_Sum(b,a),M_Sum(b,a)) + " <- eff " + "     ");


    for(int i=0; i<n; i++){
      //output.print(b.coords.get(i).x + " " + b.coords.get(i).y + " <-coords ");
      //output.println(a.linear( b.coords.get(i).x, b.coords.get(i).y ) +" <-pressure ");
      output.print("-");
    }
    output.println("");
  }
} 
  
 
/**********************************
 SaveVectorField class
 
Saves the velocity and pressure field to a text file with customizable header
These files can be quite large!

example code:

SaveVectorField data;
AudreyTest test;

void setup(){
  int resolution = 128, xLengths=6, yLengths=3, zoom = 1;
  float xStart = -4, yDist =0.2;
  test = new AudreyTest(resolution, xLengths, yLengths, xStart , yDist, zoom);
  test.update();

  data = new SaveVectorField("saved/data.txt",test.body.a.coords,test.Re,resolution, test.n,test.m);
  data.addField(test.flow.u,test.flow.p);
  data.finish();
}
***********************************/

class SaveVectorField {
  PrintWriter output;
  int m, n;

  SaveVectorField(String name, ArrayList<PVector> coords, float Re, int resolution, int n, int m) {
    this.m = m;
    this.n = n;
    output = createWriter(name);
    output.println("%% Steady field generated by a NACA foil. u, followed by v, followed by p");
    output.print("% xcoord = [");
    for (int i=0; i<coords.size(); i++) {
      output.print(coords.get(i).x +" ");
    }
    output.println("];");

    output.print("% ycoord = [");
    for (int i=0; i<coords.size(); i++) {
      output.print(coords.get(i).y +" ");
    }
    output.println("];");

    output.print("% resolution = "+ resolution);
    output.print("; n = "+ n);
    output.print("; m = "+ m);
    output.print("; Re = "+ Re);
    output.println(";");
  }


  public void addField(VectorField u, Field p) {
    for (int j=1; j<m-1; j++) {
      for (int i=1; i<n-1; i++) {
        output.print(u.x.a[i][j] +" ");
      }
      output.println(";");
    }
    for (int j=1; j<m-1; j++) {
      for (int i=1; i<n-1; i++) {
        output.print(u.y.a[i][j] +" ");
      }
      output.println(";");
    }
    for (int j=1; j<m-1; j++) {
      for (int i=1; i<n-1; i++) {
        output.print(p.a[i][j] +" ");
      }
      output.println(";");
    }
  }

  public void finish() {
    output.flush(); // Writes the remaining data to the file
    output.close(); // Closes the file
  }
} 
/**********************************
SharpField class

Holds the values and operators for a
field which represents a sharp interface

example code:

VectorField u;
SharpField p;
void setup(){
  size(800,800);
  int n=100,m=100;
  p = new SharpField(n,m);
  p.eq(1.0,10,40,60,90);
  u = new VectorField(n,m,0,0);
  for( int i=0; i<m; i++){
    for( int j=0; j<n; j++){
      u.x.a[i][j] = float(i-1)/50.0;
      u.y.a[i][j] = -float(j-1)/50.0;
    }
  }
}
void draw(){
  p.advect( 0.25, u);
  p.display(0,0.001);
  println(p.sum());
}
***********************************/

class SharpField extends Field{
  int strang = 0;
  SharpField( int n, int m, int btype, float bval ){
    super(n, m, btype, min(max(bval,0),1) );
  }
  SharpField( int n, int m){super(n, m);}
  SharpField( SharpField p){super(p);}

  public void advect( float step, Field u, Field v ){
    // determine max advection velocity
    float mx = 0;
    for( int j=1; j<m-1; j++){
    for( int i=1; i<n-1; i++){
      mx = max(abs(u.a[i][j])+abs(v.a[i][j]),mx);
    }}
    // determine CFL substep
    int it = ceil(2*mx*step);
    float substep = step/PApplet.parseFloat(it);
    // iterate split-advection method
    for (int itr = 0; itr<it; itr++){
      SharpField c = new SharpField(this);
      if(strang==0){ left( substep, u, c ); bottom( substep, v, c ); strang=1;}
      else         { bottom( substep, v, c ); left( substep, u, c ); strang=0;}
    }
  }
  public void advect( float step, Field u, Field v, Field u0, Field v0 ){
    Field us = new Field(u), vs = new Field(v);
    us.plusEq(u0); us.timesEq(0.5f);
    vs.plusEq(v0); vs.timesEq(0.5f);
    advect( step, us, vs );
  }
  
  public void left(float substep, Field u, SharpField c ){
    // flux on left face
    float[][] fx = new float[n][m];
    for( int j=1; j<m-1; j++){
      for( int i=1; i<n; i++){
        float ul = substep*u.a[i][j];
        int    s = (ul>0)?i-1:i;
        float fl = a[s][j];
        s = min(max(1,s),n-2);
        float n1 = a[s+1][j]-a[s-1][j];
        float n2 = a[s][j+1]-a[s][j-1];
        fx[i][j] = flux(ul,fl,n1,n2);
      }
    }
    // x update
    for( int j=1; j<m-1; j++){
      for( int i=1; i<n-1; i++){
        float g = (c.a[i][j]>0.5f)?1:0;
        a[i][j] -= fx[i+1][j]-fx[i][j]-g*substep*(u.a[i+1][j]-u.a[i][j]);
      }
    }
    setBC();
  }
  public void bottom( float substep, Field v, SharpField c ){
    // flux on bottom face
    float[][] fx = new float[n][m];
    for( int i=1; i<n-1; i++){
      for( int j=1; j<m; j++){
        float vb = substep*v.a[i][j];
        int    s = (vb>0)?j-1:j;
        float fb = a[i][s];
        s = min(max(1,s),m-2);
        float n1 = a[i+1][s]-a[i-1][s];
        float n2 = a[i][s+1]-a[i][s-1];
        fx[i][j] = flux(vb,fb,n2,n1);
      }
    }
    // y update
    for( int j=1; j<m-1; j++){
      for( int i=1; i<n-1; i++){
        float g = (c.a[i][j]>0.5f)?1:0;
        a[i][j] -= fx[i][j+1]-fx[i][j]-g*substep*(v.a[i][j+1]-v.a[i][j]);
      }
    }
    setBC();
  }

  public float flux( float u, float f, float n1, float n2 ){
    int sgn = (u>0)?1:-1;
    float fx;
    if(abs(n2)>abs(n1)) {
      fx = abs(u*f);
    } else if(u*n1<0){
      fx = abs(u)-(1-f);
    } else {
      fx = abs(u);
    }
    return sgn*min(f,max(0,fx));
  }
  
  public void setBC(){
    super.setBC();
    for( int i=0; i<n; i++){
      for( int j=0; j<m; j++){
        a[i][j] = min(max(a[i][j],0),1);
      }
    }
  }
  
  public SharpField smooth(){
    SharpField b = new SharpField(n,m);
    for( int j=1; j<m-1; j++ ){
    for( int i=1; i<n-1; i++ ){
        b.a[i][j] = 0.125f*(4.f*a[i][j]
        +a[i-1][j]+a[i+1][j]+a[i][j-1]+a[i][j+1]);
    }}
    b.setBC();
    return b;
  }

  public void display(){ super.display( 0, 1 ); }

}
/******************************************
 StreamPlot
 
  Draws the streamlines on top of a regular plot, using the stream function
 example code:
 
BDIM flow;
CircleBody body;
StreamPlot flood;

void setup(){
  int n=(int)pow(2,6);   // number of grid points
  int nlines = 15;       // number of streamlines
  size(400,400);         // display window size
  Window view = new Window(n,n);

  body = new CircleBody(n/3,n/2,n/8,view); // define geom
  flow = new BDIM(n,n,1.5,body);           // solve for flow using BDIM

  flood = new StreamPlot(view,flow,nlines);
  flood.range = new Scale(-.75,.75);
  flood.setLegend("vorticity");
  flood.setLineColor(0);
  flood.setLineThickness(0.1);
}
void draw(){
  body.update();
  flow.update(body);
  flow.update2();
  flood.display(flow.u.curl());
  body.display();
}
void mousePressed(){body.mousePressed();}
void mouseReleased(){body.mouseReleased();}
 ********************************************/

class StreamPlot extends FloodPlot {
  BDIM flow;
  PImage streamImage;
  int nlines;
  float thickness = 0.1f; //In fraction of spacing between streamlines
  int streamcolor = 0;

  StreamPlot(Window window, BDIM flow, int nlines) {
    super( window );
    this.flow = flow;
    this.nlines = nlines;
    streamImage = new PImage(window.dx, window.dy, ARGB);
  }
  public void setLineColor(int c) {
    this.streamcolor = c;
  }
  public void setLineThickness(float thick) {
    this.thickness = thick;
  }
  public void display ( Field a ) {
    float minv = 1e6f, maxv = -1e6f;
    super.display(a);
    Field psi = flow.u.streamFunc();

    colorMode(HSB, 360, 1.0f, 1.0f, 1.0f);
    streamImage.loadPixels();
    float psimod = (float)flow.m/nlines;
    for ( int i=0 ; i<window.dx ; i++ ) {
      float x = window.ix(i+window.x0);
      for ( int j=0 ; j<window.dy ; j++ ) {
        float y = window.iy(j+window.y0);
        float streamon = ((psi.interp(x, y)) % psimod)/psimod - 0.5f;
        if (abs(streamon)<0.5f*thickness)
          streamImage.pixels[i+j*window.dx] = color(hue(streamcolor),saturation(streamcolor),brightness(streamcolor),1-abs(streamon)/(0.5f*thickness));
        else
          streamImage.pixels[i+j*window.dx] = color(0, 0);
        float f = a.interp(x, y);
        minv = min(minv, f);
        maxv = max(maxv, f);
      }
    }
    streamImage.updatePixels();
    image(streamImage, window.x0, window.y0);
    if (legendOn)
      legend.display(minv, maxv);
  }
}


/*
Swarm class

This class and its extensions hold and update an ArrayList of Particles.

example code:

BDIM flow;
CircleBody body;
Swarm streaks;

void setup(){
  int n=(int)pow(2,6); size(400,400);
  Window view = new Window(n,n);
  body = new CircleBody(n/3,n/2,n/8,view);
  flow = new BDIM(n,n,1,body);
  streaks = new Swarm( view );  // adds when clicked or dragged
//  streaks = new SourceSwarm( view, new PVector(0,33) );
//  streaks = new FieldSwarm( view, 5000 );
//  streaks = new InletSwarm( view, 5000 );
}
void draw(){
//  background(200); // turn this on/off
  flow.update(); flow.update2();
  streaks.bodyColor = color(random(0,255),random(0,255),random(0,255));
  streaks.update(flow);
  streaks.display();
  body.display();
}
void mousePressed(){streaks.mousePressed();}
void mouseDragged(){streaks.mousePressed();}

**************************/

class Swarm{
  ArrayList<Particle> pSet;
  Window window;
  int bodyColor = color(0);
  int imax=1000, lifeSpan=200;
  boolean points=false, lines=true;

  Swarm( Window window ){ 
    pSet = new ArrayList<Particle>();
    this.window = window;
  }

  public void update(VectorField u, VectorField u0 , float dt){
    remove();
    add( u, u0, dt);
    for( Particle p: pSet) p.update( u, u0, dt );
    println("Swarm size:"+pSet.size());
  }
  public void update(BDIM flow){ update(flow.u,flow.u0,flow.dt); }

  public void remove(){
    Iterator<Particle> pIter = pSet.iterator();
    while ( pIter.hasNext() ){
      if( pIter.next().dead() ) pIter.remove();
    }
  }

  public void  add(VectorField u, VectorField u0 , float dt){
    return;
  }
  
  public void display(){
    if(lines)  for( Particle p: pSet) p.display();
    if(points) for( Particle p: pSet) p.displayPoint();
  }
  
  public void mousePressed() {
    float x = window.ix(mouseX), y = window.iy(mouseY);
    pSet.add( new Particle( x, y, bodyColor, window, lifeSpan ) );
  }

}


// Add particles uniformly over the window and upstream ! good for streamlines
class FieldSwarm extends Swarm{
  
  FieldSwarm( Window window , int imax ){
    super( window );
    this.imax = imax;
  }

  public void add(VectorField u, VectorField u0 , float dt){
    float upstream = u.x.bval*lifeSpan*dt;
    int i0 = 1-window.px(upstream);
    for( int i=0; pSet.size()<imax & i<imax/lifeSpan; i++) {
      float x = random(i0,width), y = random(0,height);
      x = window.ix((int)x); y = window.iy((int)y);
      pSet.add( new Particle( x, y, bodyColor, window, lifeSpan ) );
    }
  }
}

// Add particle at a source point ! good for streaklines
class SourceSwarm extends Swarm{
  PVector p;

  SourceSwarm( Window window, PVector p ){
    super( window );
    this.p = new PVector(p.x,p.y);
    points = true; lines = false;
  }
  
  public void add(VectorField u, VectorField u0 , float dt){
    Particle q = new Particle( p.x, p.y, bodyColor, window, lifeSpan );
    int n = imax/lifeSpan;
    for( int i=0; i<n ; i++){
      pSet.add( new Particle( q.x.x, q.x.y, bodyColor, window, lifeSpan ) );
      q.update( u, u0, -dt/(float)n ); // advect a partial step back
    }
  }
  
  public void mousePressed() {
    p.x = window.ix(mouseX);
    p.y = window.iy(mouseY);
  }
}

// Add particles along the inlet ! good for colored path areas
class InletSwarm extends Swarm{
  
  InletSwarm( Window window, int imax ){
    super( window );
    this.imax = imax;
  }
  
  public void add(VectorField u, VectorField u0 , float dt){
    for( int i=0; pSet.size()<imax & i<max(1,imax/lifeSpan) ; i++) {
      float x = 0, y = random(0,height);
      int c = (y<height/2+1)?color(0xff00FF33):color(0xff215E21);
//      color c = bodyColor;
      x = window.ix((int)x); y = window.iy((int)y);
      pSet.add( new Particle( x, y, c, window, lifeSpan ) );
    }
  }
}

// Extend SourceSwarm to fill in gaps between points. Even better for streaklines
class StreakSwarm extends SourceSwarm{
  Particle pp;

  StreakSwarm( Window window, PVector p, int bodyColor ){
    super( window, p );
    lifeSpan *= 10;
    this.bodyColor = bodyColor;
    pSet.add(new Particle( p.x, p.y, bodyColor, window, lifeSpan));
    pp = new Particle( p.x, p.y, bodyColor, window, lifeSpan);
  }
  
  public void add(VectorField u, VectorField u0 , float dt){
    ListIterator<Particle> pIter = pSet.listIterator();
    while ( pIter.hasNext() ){
      Particle a = pIter.next();
      Particle b = pp;
      if( pIter.hasNext() ) b = pSet.get(pIter.nextIndex());
      int d2 = window.pdx(sq(a.x.x-b.x.x)+sq(a.x.y-b.x.y));
      if(d2>9) {
        pIter.add( new Particle( 0.5f*(a.x.x+b.x.x), 0.5f*(a.x.y+b.x.y), bodyColor, window, lifeSpan, (a.step+b.step)/2 ) );
      }else if(d2<1) {
        pIter.remove();
      }
    }
  }
}
/**********************************
TwoPhase class

Extends BDIM by adding two-phase modeling

This means we need to track the free interface and apply the 
variable density to the pressure gradient

example code:

TwoPhase flow;
void setup(){
  int n=(int)pow(2,7), m=(int)pow(2,6); size(800,400);
  flow = new TwoPhase(n,m,2,0.01);
  flow.f.eq(0,m,n+2,0,m+2);
  flow.u = new VectorField(n+2,m+2,0,0);
}
void draw(){
  flow.update();
  flow.update2();
  flow.f.display();
}
***************************************/

class TwoPhase extends BDIM{
  FreeInterface f,f0;

  TwoPhase( int n, int m, float dt, Body body, float nu, boolean QUICK, float g, float gamma ){
    super( n, m, dt, body, nu, QUICK);
    f  = new FreeInterface( this.n, this.m, gamma );
    f0 = new FreeInterface( this.n, this.m, gamma );
    this.g.y = g;
  }
  TwoPhase( int n, int m, float dt, float g ){
    this( n, m, dt, new CircleBody(-10.f,-10.f,0.f,new Window()), 0, false, g, 0.1f);
  }

  public void update(){
    f.strang = (f.strang+1)%2; // strang operator splitting
    rhoi.eq(f.rho().inv());
    f0.eq(f);
    f.advect( dt, u0 );
    super.update();
  }
  public void update2(){
    rhoi.eq(f.rho().inv());
    c.eq(del.times(rhoi.times(dt))); // need to recompute
    f.eq(f0);
    f.advect( dt, u0, u );
    super.update2();
  }
}
/*************************
UnsteadyLiftControl test;
SaveData dat;
float maxT = 10;
float numframes = 1000;
float frame = 0;
String datapath = "";
BufferedReader reader;
float[] yd = {0.5, 0.25, -0.25, 0.5};

void setup() {
  int Re = 6000;
  int resolution = 32, xLengths=7, yLengths=7, xStart = 3, zoom = 2;

  test = new UnsteadyLiftControl(resolution, xLengths, yLengths, xStart, zoom, Re, true);

  mm = new MovieMaker(this, width, height, datapath+"control.mov", 30);
  dat = new SaveData(datapath+"control.txt", test.foil.coords, resolution, xLengths, yLengths, int(zoom*10));
  
  maxT = maxT*resolution;
}
void draw() {
  int i = floor(test.t/maxT*yd.length);
  if (i>=yd.length){i = yd.length-1;}
  test.setyd(yd[i]);
  test.update();
  test.display();
  dat.addData(test.t, test.foil.pressForce(test.flow.p), test.foil, test.flow.p);

  if (frame<test.t*numframes/maxT) {
    frame = frame+1;
    mm.addFrame();
  }

  if (test.t>=maxT) {
    dat.finish();
    mm.addFrame();
    mm.finish();
    exit();
  }
}
 ***********************/

class UnsteadyLiftControl {
  final int n, m;
  float dt = 0, t = 0, dto = 0;
  float a = 5*PI/180, w = 2*PI;
  float yd = 0;
  
  float[] Xhat = {0, 0, 0, 0};
  float[] A0 = {-0.3414f, 1, 0, 0}, A1 = {-0.01582f, 0, 0, 0}, A2 = {PI, 0, 0, 0}, A3 = {PI/2, 0, 1, 0};
  float[] B = {0, 0, 0, 1}, Ct = {0.09846f, 0.00757f, 0.5177f*PI, PI/4+0.5177f*PI/2};
  float D = PI/16, alph = 4.2640f;
  float[] Kt = {0.4049f, 0.0314f, 7.1122f, 7.8715f}, L = {2.4923f, 0.1078f, 17.449f, 16.8507f};
  int nX = 4;
  
  float th = 0, thdot = 0, thddot= 0;
  float lift=0;
  int resolution, zoom;

  NACA foil; 
  BDIM flow; 
  FloodPlot flood, flood2; 
  Window window;

  UnsteadyLiftControl( int resolution, int xLengths, int yLengths, int xStart, int zoom, int Re, boolean QUICK) {
    this.resolution = resolution;
    this.zoom = zoom;
    n = xLengths*resolution+2;
    m = yLengths*resolution+2;
    //thdot = a*w;

    int w = PApplet.parseInt(zoom*(n-2)), h = PApplet.parseInt(zoom*(m-2));
    size(w, h);
    window = new Window(n, m);

    foil = new NACA(xStart*resolution, m/2, resolution, .12f, .25f, window);
    
    foil.rotate(0);
    foil.translate(xLengths, -yLengths);
    foil.translate(0,0);
    flow = new BDIM(n, m, dt, foil, (float)resolution/Re, QUICK);

    flood = new FloodPlot(window);
    flood.range = new Scale(-1, 1);
    flood.setLegend("vorticity");
    flood.setColorMode(1); 
    foil.setColor(0xffCCCCCC);

    flood2 = new FloodPlot(window);
    flood.range = new Scale(-0.5f, 0.5f);
    flood2.setLegend("pressure");
  }
  public void setSys(float[] A1, float[] A2, float[] A3, float[] A4, float[] B, float[] Ct, float D, int nX) {
    this.Xhat = new float[nX];
    this.nX = nX;
    this.A1 = A1;
    this.A2 = A2;
    this.A3 = A3;

    this.B = B;
    this.Ct = Ct;
    this.D = D;
  }
  public void setGains(float[] Kt, float[] L, int nX) {
    this.L = L;
    this.Kt = Kt;
  }

  public float controller(float[] Xhat, float yd) {
    //Takes Xhat, computes Thetaddot
    return -Kt[0]*Xhat[0]-Kt[1]*Xhat[1]-Kt[2]*Xhat[2]-Kt[3]*Xhat[3]+ alph*yd;
    //return -w*w*a*sin(t/resolution*w);
  }
  public float[] estimator(float y, float u, float[] Xhat_in) {
      float Xhatdot;
      float[] Xhatnew = new float[nX];
      float yhat = Ct[0]*Xhat_in[0] + Ct[1]*Xhat_in[1] + Ct[2]*Xhat_in[2] + Ct[3]*Xhat_in[3] + D*u;
      for(int i=0; i<nX; i++){
        Xhatdot = A0[i]*Xhat_in[0]+A1[i]*Xhat_in[1]+A2[i]*Xhat_in[2]+A3[i]*Xhat_in[3]+B[i]*u+L[i]*y-L[i]*yhat;
        Xhatnew[i] = Xhat_in[i]+dt/resolution*Xhatdot;
      }
      return Xhatnew;
  }
  public void setyd(float yd){
    this.yd = yd;
  }

  public void update() {
    dto = dt;
    if (flow.QUICK) {
      dt = flow.checkCFL();
      flow.dt = dt;
    }
    float dtr = dt/resolution;
    
    PVector pforce = foil.pressForce(flow.p);
    lift = pforce.y;
 
    // Do first Euler step
    float thddot0 = controller(Xhat,yd);
    float dth0 = thdot*dtr;
    float dthdot0 = thddot0*dtr;
    
    //Integrate
    float thdot1 = thdot+dthdot0;
    float[] Xhat1 = estimator(lift,thddot0, Xhat);

    //Do Second Euler Step
    float thddot1 = controller(Xhat1,yd);
    float dth1 = thdot1*dtr;
    float dthdot1 = thddot1*dtr;
    
    //Integrate
    th = th + 0.5f*(dth0+dth1);
    thdot = thdot + 0.5f*(dthdot0+dthdot1);
    thddot = 0.5f*(thddot0+thddot1);
    Xhat = estimator(lift,thddot, Xhat);
    
    foil.rotate(-foil.phi+th);
    println("Theta: "+th*180/PI + "  Thetadot: "+thdot*180/PI + "  Thetaddot: "+thddot*180/PI + "  dt:" + dt/resolution);
    println("Xhat: "+Xhat[0]*180/PI+"  "+Xhat[1]*180/PI + "  "+Xhat[2]*180/PI + "  "+Xhat[3]*180/PI);
    
    if (mousePressed==true){
    flow.u.x.a[mouseX/zoom][mouseY/zoom-2] += 1;
    flow.u.x.a[mouseX/zoom][mouseY/zoom-1] += 2;
    flow.u.x.a[mouseX/zoom][mouseY/zoom] += 2;
    flow.u.x.a[mouseX/zoom][mouseY/zoom+1] += 2;
    flow.u.x.a[mouseX/zoom][mouseY/zoom+2] += 1;
    }
     
    flow.update(foil);
    flow.update2(foil);
    t += dt;
  }
  public void display() {
    flood.display(flow.u.curl());
    foil.display();
    foil.displayVector(foil.pressForce(flow.p));
    
    PFont font = loadFont("Dialog.bold-14.vlw");
      int x0 = window.x0, x1 = window.x0+window.dx;
      int y0 = window.y0, y1 = window.y0+window.dy;
      int spacing = 20;
      textFont(font);
      textAlign(CENTER,BASELINE);
      fill(0xff000000);
      text("t = "+ nfs(t/resolution,2,2),0.5f*(x0+x1),y1-spacing*2);
      textAlign(RIGHT,BASELINE);
      text("Theta: "+ nfs(th*180/PI,2,2),x1-spacing,y1-2*spacing);
      text("Liftset: "+ nfs(yd,2,2),x1-spacing,y1-spacing);
  }
}
/**********************************
VectorField class

Holds the values and operators for a
vector field

Example code:
void setup(){
  size(400,400);
  noStroke();
  int N = 130;
  VectorField u = new VectorField(N,N,1,-0.5);
  u.x.eq(0.,40,50,40,75);
  VectorField c = new VectorField(N,N,0,0);
  c.x.eq(1); c.y.eq(1); c.setBC();
  Field p = new Field(N,N);
//  u.project(c,p);  // uncomment to see the effect of projection on the divergence.
  u.divergence().display(-.1,.1);
}
***********************************/

class VectorField{
  Field x,y;
  int n, m;
  float CF=1.f/6.f, S=10.f;  // QUICK parameters

  VectorField( int n, int m, float xval, float yval ){
    this.n = n;
    this.m = m;
    x = new Field( n, m, 1, xval );
    y = new Field( n, m, 2, yval );
  }
  VectorField( Field x, Field y ){
    n = x.n;
    m = x.m;
    this.x = new Field(x);
    this.y = new Field(y);
  }
  VectorField( VectorField b ){this( b.x, b.y );}
  
  public void setBC(){
    x.setBC(); 
    y.setBC(); 
  }

  public VectorField normalGrad(VectorField wnx, VectorField wny){
    VectorField g = new VectorField(n,m,0,0);
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      g.x.a[i][j] = 0.5f*(wnx.x.a[i][j]*(x.a[i+1][j]-x.a[i-1][j])+wny.x.a[i][j]*(x.a[i][j+1]-x.a[i][j-1]));
      g.y.a[i][j] = 0.5f*(wnx.y.a[i][j]*(y.a[i+1][j]-y.a[i-1][j])+wny.y.a[i][j]*(y.a[i][j+1]-y.a[i][j-1]));
    }}
    return g; 
  }

  public Field divergence (){
    // returns div{this} for unit cells
    Field d = new Field( n, m );
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      d.a[i][j] = x.a[i+1][j]-x.a[i][j]+
                  y.a[i][j+1]-y.a[i][j];
    }}
    return d;
  }

  public Field ke (){
    // returns 0.5*{this-bval}^2 for unit cells
    Field d = new Field( n, m );
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      d.a[i][j] = (sq(x.a[i+1][j]+x.a[i][j]-2.f*x.bval)+
                   sq(y.a[i][j+1]+y.a[i][j]-2.f*y.bval))*0.125f;
    }}
    return d;
  }

  public Field curl (){
    // returns curl{this} located at cell corner (btype=3)
    Field d = new Field( n, m, 3, 0 );
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
      d.a[i][j] = x.a[i][j-1]-x.a[i][j]+
                  y.a[i][j]-y.a[i-1][j];
    }}
    return d;
  }

  public Field Qcrit (){
    Field q = new Field( n, m );
    for ( int i=1 ; i<n-1 ; i++ ) {
    for ( int j=1 ; j<m-1 ; j++ ) {
    float dudx = 0.5f*(x.a[i+1][j]-x.a[i-1][j]);
    float dudy = 0.5f*(x.a[i][j+1]-x.a[i][j-1]);
    float dvdx = 0.5f*(y.a[i+1][j]-y.a[i-1][j]);
    float dvdy = 0.5f*(y.a[i][j+1]-y.a[i][j-1]);
    q.a[i][j] = dudx*dvdy-dvdx*dudy;
    }}
    return q;
  }
  public Field streamFunc() {
    //Integrates the flow field to get a stream function
    Field psi = new Field( n, m );
    float[][] s1 = new float[n][m];
    float[][] s2 = new float[n][m];

    //Integrate from top left corner
    for (int i=0 ; i<n ; i++ )
      s1[i][0] = 0;
    for (int i=0 ; i<n ; i++ )
      for (int j=1 ; j<m ; j++ )
        s1[i][j] = s1[i][j-1]+0.5f*(x.a[i][j-1]+x.a[i][j]);

    //Integrate from lower right corner
    s2[n-1][m-1]=0;
    for (int i=n-2; i>=0; i--)
      s2[i][m-1] = 0;
    for (int i=0 ; i<n ; i++ )
      for (int j=m-2 ; j>=0; j-- )
        s2[i][j] = s2[i][j+1]-0.5f*(x.a[i][j+1]+x.a[i][j]);

    //Average both solutions
    float basepsi = s2[0][0];
    for (int i=0 ; i<n ; i++ )
      for (int j=0 ; j<m ; j++ )
        psi.a[i][j] = 0.5f*(s1[i][j] + s2[i][j]-basepsi);    
    return psi;
  }

  public Field project ( VectorField coeffs, Field p, Field s ){
    /* projects u,v onto a divergence-free field using
         div{coeffs*grad{p}} = div{u}  (1)
         u -= coeffs*grad{p}           (2)
       and returns the field p. all FDs are on unit cells */
    p = MGsolver( 20, new PoissonMatrix(coeffs), p , s );
    p.plusEq(-1*p.sum()/(float)((n-2)*(m-2)));
    VectorField dp = p.gradient();
    x.plusEq(coeffs.x.times(dp.x.times(-1)));
    y.plusEq(coeffs.y.times(dp.y.times(-1)));
    setBC();
    return p;
  }
  public Field project ( VectorField coeffs, Field p ){ return project(  coeffs, p, this.divergence() ); }

  public void display( float unit, int skip){
    stroke(0xff993333);
    float DX = width/(float)n;
    float DY = height/(float)m;
    for ( int i=0 ; i<n ; i+=skip ) {
    for ( int j=0 ; j<m ; j+=skip ) {
      float px = i*DX;
      float py = j*DY;
      arrow(px,py,px+DX*unit*x.a[i][j],py+DY*unit*y.a[i][j]);
    }}
    noStroke();
  }  
  private void arrow(float x1, float y1, float x2, float y2) {
    float a = atan2(x1-x2, y2-y1);
    float b = 0.1f*mag(x1-x2, y2-y1);
//    if(b<.1) return;
    line(x1, y1, x2, y2);
    pushMatrix();
      translate(x2, y2);
      rotate(a);
      line(0, 0, -b, -b);
      line(0, 0,  b, -b);
    popMatrix();
  } 
  
   public void AdvDif(VectorField u0, float dt, float nu) {
     VectorField v = new VectorField(this);
     for ( int j=1; j<m-1; j++) {
      for ( int i=1; i<n-1; i++) {
        v.x.a[i][j] = (advection(x, i, j) + nu*diffusion(x, i, j))*dt+u0.x.a[i][j];
        v.y.a[i][j] = (advection(y, i, j) + nu*diffusion(y, i, j))*dt+u0.y.a[i][j];
      }
    }
    this.eq(v);   
  }

  public float advection (Field b, int i, int j) {  
    float uo, ue, vs, vn;
    if (b.btype == 1) {
      uo = 0.5f*(x.a[i-1][j]+x.a[i][j]);
      ue = 0.5f*(x.a[i+1][j]+x.a[i][j]);
      vs = 0.5f*(y.a[i][j]+y.a[i-1][j]);
      vn = 0.5f*(y.a[i][j+1]+y.a[i-1][j+1]);
    }
    else {
      uo = 0.5f*(x.a[i][j-1]+x.a[i][j]);
      ue = 0.5f*(x.a[i+1][j-1]+x.a[i+1][j]);
      vs = 0.5f*(y.a[i][j-1]+y.a[i][j]);
      vn = 0.5f*(y.a[i][j]+y.a[i][j+1]);
    }
    return ((uo*bho(b, i, j, -1, 0, uo) - ue*bho(b, i, j, 1, 0, ue)) + (vs*bho(b, i, j, 0, -1, vs) - vn*bho(b, i, j, 0, 1, vn)));
  }

  public float diffusion (Field b, int i, int j) {
    return b.a[i+1][j] + b.a[i][j+1] - 4*b.a[i][j] + b.a[i-1][j] + b.a[i][j-1];
  }

  public float bho(Field b, int i, int j, int d1, int d2, float uf) {
    float bf =  0.5f*(b.a[i+d1][j+d2]+b.a[i][j]); 
    if (d1*uf<0){
     i += d1; 
     d1 = -d1;
    }
    if (d2*uf<0){
     j += d2;
     d2 = -d2;
    } 
    if ( i>n-2 || i<2 || j>m-2 || j<2 ) return bf;
    float bc = b.a[i][j];
    float bd = b.a[i+d1][j+d2];
    float bu = b.a[i-d1][j-d2];
    bf -= CF*(bd-2*bc+bu);
    float b1 = bu+S*(bc-bu);
    return med(bf, bc, med(bc, bd, b1));
  }

  public float med(float a, float b, float c) {
    return(max(min(a, b), min(max(a, b), c)));
  }

  public float CFL(float nu) {
    float b = abs(x.a[0][0])+abs(y.a[0][0]);
    float c;
    for ( int i=1; i<n-1; i++) {
      for ( int j=1; j<m-1; j++) { 
        c = abs(x.a[i][j])+abs(y.a[i][j]);
        if (c>b) b=c;
      }
    }
    return 1.f/(b+3.f*nu);
  }
  
  public VectorField times( VectorField b){
    VectorField g = new VectorField(this);
    g.timesEq(b);
    return g;
  }
  
  public VectorField times( float b){
    VectorField g = new VectorField(this);
    g.timesEq(b);
    return g;
  }
  
  public VectorField plus( VectorField b){
    VectorField g = new VectorField(this);
    g.plusEq(b);
    return g;
  }
  
  public VectorField minus( VectorField b){
    VectorField g = new VectorField(this);
    g.minusEq(b);
    return g;
  }
  
  public VectorField plus( float b){
    VectorField g = new VectorField(this);
    g.plusEq(b);
    return g;
  }  

  public VectorField inv(){ 
    VectorField g = new VectorField(this);
    g.invEq();
    return g;
  }

  public VectorField laplacian(){
    return new VectorField(x.laplacian(),y.laplacian());
  }
  
  public void eq( VectorField b ){ x.eq(b.x); y.eq(b.y);}
  public void eq( float b ){ x.eq(b); y.eq(b);}
  public void timesEq( VectorField b ){ x.timesEq(b.x); y.timesEq(b.y);}
  public void timesEq( float b ){ x.timesEq(b); y.timesEq(b);}
  public void plusEq( VectorField b ){ x.plusEq(b.x); y.plusEq(b.y);}
  public void plusEq( float b ){ x.plusEq(b); y.plusEq(b);}  
  public void plusEq( PVector b ){ x.plusEq(b.x); y.plusEq(b.y);}  
  public void minusEq( VectorField b ){ x.minusEq(b.x); y.minusEq(b.y);}  
  public void advect( float dt, VectorField b ){ x.advect(dt,b); y.advect(dt,b);}
  public void advect( float dt, VectorField b, VectorField b0 ){ x.advect(dt,b,b0); y.advect(dt,b,b0);}
  public void invEq(){ x.invEq(); y.invEq();}  
}
/*************************
 Vortex class
 
 This is an example class for initializing the flow with one or more Rankine 
 vortices.  In the example code, a vortex pair is initialized in a quiescent 
 flow and then moves with its self-induced velocity.
 
 Example code:
//INPUT PARAMETERS_______________________________________________________________________
int resolution = 8;            // number of grid points spanning vortex diameter
int xLengths = 16;             // (streamwise length of computational domain)/(resolution)
int yLengths = 8;              // (transverse length of computational domain)/(resolution)
int area = 300000;             // window view area
int Re = 1000;                 // Reynolds number
float q = 1;                   // vortex circulation
float sep = 1;                 // (separation between opposite sign vortices)/(core diameter)
//_______________________________________________________________________________________

Vortex test;

void settings(){
  float s = sqrt(area*xLengths/yLengths);
  size((int)s, (int)s*yLengths/xLengths);
}
void setup() {
  test = new Vortex(resolution, Re, xLengths, yLengths, q, sep);
}

void draw() {
  test.update(); 
  test.display();
}

void keyPressed(){exit();}
***********************/


class Vortex {
  BDIM flow;
  boolean QUICK = true, order2 = true;
  int n, m, resolution;
  float dCore, q, sep, xc, yc, dt=1;
  CircleBody body;
  FloodPlot flood;
  VectorField u0;

  Vortex(int resolution, int Re, int xLengths, int yLengths, float q, float sep) {
    this.resolution = resolution;
    n=xLengths*resolution+2;
    m=yLengths*resolution+2;
    Window view = new Window(0, 0, n, m);
    this.dCore = resolution;
    this.q = q;
    this.sep = sep;
    this.u0 = new VectorField(n, m, 0, 0);
    xc = 0.2f*n;
    yc = 0.5f*m;
  
    //set up an initial velocity field of a Rankine vortex pair
    for ( int i=1 ; i<n-1 ; i++ ) {
      for ( int j=1 ; j<m-1 ; j++ ) {
        for ( int btype=1 ; btype<3 ; btype++ ) {
          float x = i;
          float y = j;
          if (btype==1) {
            x -= 0.5f;
          }
          if (btype==2) {
            y -= 0.5f;
          }
          //first vortex
          if (btype==1) {
            u0.x.a[i][j] += vortexX(x, y, xc, (yc - sep*dCore/2), -1);
          }
          if (btype==2) {
            u0.y.a[i][j] += vortexY(x, y, xc, (yc - sep*dCore/2), -1);
          }
          //second vortex
          if (btype==1) {
            u0.x.a[i][j] += vortexX(x, y, xc, (yc + sep*dCore/2), 1);
          }
          if (btype==2) {
            u0.y.a[i][j] += vortexY(x, y, xc, (yc + sep*dCore/2), 1);
          }
        }
      }
    }
    
    flow = new BDIM(n-2, m-2, 0, u0, (float)resolution/Re, QUICK);

    flood = new FloodPlot(view); 
    flood.range = new Scale(-.5f, .5f);
    flood.setLegend("vorticity");
  }

  // the integer pos specifies if the circulation is positive or negative
  public float vortexX( float x, float y, float xc, float yc, int pos) {
    float c;
    if ((sq(x-xc) + sq(y-yc)) <= sq(dCore/2)) {
      c = -pos*q*(y-yc);
    } 
    else {
      c = -pos*q*sq(dCore/2)/( sq(x-xc) + sq(y-yc) )*(y-yc);
    }
    return c;
  }  

  public float vortexY( float x, float y, float xc, float yc, int pos) {
    float c;
    if ((sq(x-xc) + sq(y-yc)) <= sq(dCore/2)) {
      c = pos*q*(x-xc);
    } 
    else {
      c = pos*q*sq(dCore/2)/( sq(x-xc) + sq(y-yc))*(x-xc);
    }  
    return c;
  }

  public void update() {
    if (QUICK) {
      dt = flow.checkCFL();
      flow.dt = dt;
    }
    
    flow.update();
    if (order2) {
      flow.update2();
    }
  }

  public void display() {
    flood.display(flow.u.curl());
  }
}
/********************************************
  Class to hold display window parameters
    and functions to convert from
    pixels to other units

Example code:
void setup(){
  size(300,450);
  Window mine = new Window(1,1,10,15,40,80,200,300);
  rect(40,80,200,300);
  stroke(255,0,0);fill(0,255,255);
  rect(mine.px(0.5),mine.py(0.5),mine.x.r,mine.y.r);
  rect(mine.px(9.5),mine.py(14.5),mine.x.r,mine.y.r);
  line(mine.px(1),mine.py(1),mine.px(10),mine.py(15));
  rect(mine.px(mine.ix(0)),mine.py(mine.iy(449)),50,-50);
  rect(mine.px(mine.ix(299)),mine.py(mine.iy(0)),-50,50);
}
*********************************************/

class Window{
  Scale x,y;
  int x0,y0,dx,dy;
  
  Window(){ this( 0.f, 0.f, 1.f, 1.f, 0, 0, width, height );}
  Window( int n, int m){ this( 1, 1, n-2, m-2, 0, 0, width, height );}
  Window( int n0, int m0, int dn, int dm){this( n0, m0, dn, dm, 0, 0, width, height );}
  Window( int n0, int m0, int dn, int dm, int x0, int y0, int dx, int dy){
    this(n0-0.5f,m0-0.5f,dn,dm,x0,y0,dx,dy);}
  Window( int n0, float m0, int dn, float dm, int x0, int y0, int dx, int dy){
    this(n0-0.5f,m0,dn,dm,x0,y0,dx,dy);}
  Window( float n0, float m0, float dn, float dm, int x0, int y0, int dx, int dy){
    x = new Scale(n0,n0+dn,x0,x0+dx);
    y = new Scale(m0,m0+dm,y0,y0+dy);
    this.x0 = x0;
    this.y0 = y0;
    this.dx = dx;
    this.dy = dy;
  }
  public float ix(int i){ return x.in((float)i);}
  public float iy(int i){ return y.in((float)i);}
  public float idx(int i){ return i/x.r;}
  public float idy(int i){ return i/y.r;}
  public int px(float i){ return (int)(x.out(i));}
  public int py(float i){ return (int)(y.out(i));}
  public int pdx(float i){ return (int)(x.r*i);}
  public int pdy(float i){ return (int)(y.r*i);}
  public boolean inside( int x, int y ){
    return( x>=x0 && x<=x0+dx && y>=y0 && y<=y0+dy );
  }
}

class Scale{
  float inS,inE,outS,outE,r;

  Scale( float outS, float outE ){ this(0,1,outS,outE);}
  Scale( float inS, float inE, float outS, float outE ){
    this.inS  = inS;
    this.inE  = inE;
    this.outS = outS;
    this.outE = outE;
    r = (outE-outS)/(inE-inS);
  }
  public float outB( float in ){ return out(min(max(in,inS),inE));}
  public float out( float in ){ return (in-inS)*r+outS;} 
  public float in( float out ){ return (out-outS)/r+inS;}
}
  public void settings() {  size(1600, 800); }
  static public void main(String[] passedArgs) {
    String[] appletArgs = new String[] { "ThreePartFish" };
    if (passedArgs != null) {
      PApplet.main(concat(appletArgs, passedArgs));
    } else {
      PApplet.main(appletArgs);
    }
  }
}
