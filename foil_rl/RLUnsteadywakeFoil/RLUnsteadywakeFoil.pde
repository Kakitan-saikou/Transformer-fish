import java.util.*;
import org.apache.xmlrpc.*;
import java.util.concurrent.*;
import java.util.concurrent.Semaphore;
import com.alibaba.fastjson.JSONObject;
FoilUnsteadywake test;
SaveData dat;
float maxT;
PrintWriter output;

int Re = 1173;
float chord = 1.0;
//float dAoA = 25*PI/180, uAoA = dAoA;
int resolution = 32, xLengths = 16, yLengths = 12, zoom = 3;
int picNum = 10;
float tStep = .005;
String datapath = "saved" + "/";
int EpisodeNum = 1;
int EpisodeTime = 50, ActionTime = 0; //TODO: actionTime = 5
float CT = 0, CL = 0, CP = 0, forceX = 0;
float AvgCT = 0, Avgeta = 0;
float AoA = 0, A = 0, Xf = 0;
float y = 0, angle = 0;
String p;
XmlRpcClient client;
WebServer server;
float[] ycoords;

int StepTime = 10, StepCount = 1;// used for adjust Ct avg length

float nextactionAoA=0,nextactionA=0, nextactionXf = 0;
float formerA = 0, formerAoA = 0,formerAvel = 0, formerAoAvel = 0;
float[] NextAction = {nextactionA,nextactionAoA, nextactionXf};
float[] FormalAction = {formerA,formerAvel,formerAoA, formerAoAvel};


//remote info exchange
ArrayList state = new ArrayList();
float[] action = new float[3];
float reward_buffer = 0;
Boolean done = false;
int reset_flag = 0;

// Semaphore to provide asynchronization
Semaphore action_sem = new Semaphore(0);
Semaphore state_sem = new Semaphore(0);

void settings()
{
  size(zoom * xLengths * resolution, zoom * yLengths * resolution);

}

void setup()
{
  //surface.setVisible(false);
  //println(args);
  println(8686);


  try
  {

    //server = StartServer(int(args[0]));
    server = StartServer(8686);
    datapath = datapath+args[1]+"/";

  }
  catch (Exception ex)
  {
    println(ex);
  }
  setUpNewSim(EpisodeNum);
}
void draw(){
  if (test.t <= EpisodeTime){
    // TODO: check reset and done [notice: reset_flag will be changed by server]
    if(reset_flag == 1){
     dat.finish();
     output.close();
     EpisodeNum +=1;
     A = 0;
     AoA = 80;
     Xf = 0;
     setUpNewSim(EpisodeNum);
     test.t = 0;
     done = false;
     reset_flag = 0;

     String Sp = SparsePressure(test);
     String state_cn = multy_state(y,angle,0,0,0,0, Sp);
     state.clear();
     state.add(state_cn);
    }
    if(done == true){// done == true stop sim
        //continue
        dat.finish();
        output.close();

    }
    if (test.t<ActionTime){
      A = 80;
      AoA = 0;
      test.update2(AoA, A,Xf);
      test.display();
      PVector forces = test.foil.pressForce(test.flow.p);
      CT = -forces.x / resolution * 2; //Change to Instantaneous value from increment value
      //ycoords = ycoords(test);
      angle = (test.foil.phi - PI) / PI * 180.;
      PVector coordinate = test.foil.xc;
      y = coordinate.y - yLengths * resolution / 2.;
      PVector deltaxy = test.foil.dxc;
      float velocityphi = test.foil.dphi*resolution/test.flow.dt;
      float velocity = - deltaxy.y*resolution/test.flow.dt;// negative sign due to the coordinate change
      float velocitx = deltaxy.x*resolution/test.flow.dt;
      float M = test.foil.pressMoment(test.flow.p);
      CP  = ((M*velocityphi/resolution)-(forces.y*velocity/resolution)); // This is right formular
      //float Eta  = -forces.x/CP; // we can multipe a number to rescale eta
      String Sp = SparsePressure(test);
      float Eta = 0;
      if (CP == 0){
       Eta = -5.;
      }
      else {
        Eta  = -forces.x/CP; // we can multipe a number to rescale eta
      }

      String state_cn = multy_state(y,angle,velocity,velocitx,velocityphi,Eta, Sp);
      state.clear();
      state.add(state_cn);

    }
    else if(test.t>=ActionTime){
      
     
    //FormalAction = NextAction;
    //System.out.println("[debug]before action...");
    NextAction = callActionByRemote();
    //System.out.println("[debug]after action...");
      
    
    nextactionA = NextAction[0];
    nextactionAoA = NextAction[1];
    nextactionXf = NextAction[2];
    
    
    
    /* position control command
    //Action limitation for pitch and heave
    // Each action A<1.2, AoA<1.5; (A and AoA are relative position change value
    A = A + nextactionA; // 20-300 heave limit
    AoA = AoA + nextactionAoA; // 70 degree pitch limit
    if (A>=250||A<=50){
      A = A - 2*nextactionA;
    }
    if (abs(AoA)>=70){
      AoA = AoA - 2*nextactionAoA;
    }
    */
    
    // velocity control command
    // TODO: Add motion normalization in Python
    A = nextactionA; // the max change is 0.6
    AoA = nextactionAoA; // the max change is 5
    Xf = nextactionXf;
    PVector foilcoordinate = test.body.bodyList.get(0).xc;
    PVector ccoordinate = test.body.bodyList.get(1).xc;
    
    if (foilcoordinate.x-ccoordinate.x<30 && Xf>0){
      Xf = -Xf;
    }// For not collision, may confuse RL 
    
    test.update2(AoA, A, Xf);
    test.display();
    PVector forces = test.body.bodyList.get(0).pressForce(test.flow.p);
    CT = -forces.x / resolution * 2;
    ycoords = ycoords(test);
    angle = (test.body.bodyList.get(0).phi - PI) / PI * 180.;
    y = foilcoordinate.y - yLengths * resolution / 2.;
    PVector deltaxy = test.body.bodyList.get(0).dxc;
    float velocityphi = test.body.bodyList.get(0).dphi*resolution/test.flow.dt;
    float velocity = - deltaxy.y*resolution/test.flow.dt;
    float velocitx = deltaxy.x*resolution/test.flow.dt;
    float M = test.body.bodyList.get(0).pressMoment(test.flow.p);
    forceX = forces.x;
    // TODO: fix bug Eta might be a global variable.
    CP  = ((M*velocityphi/resolution)-(forces.y*velocity/resolution));
    float Eta = 0;
    if (CP == 0){
       Eta = -5.;
    }
    else {
       Eta  = -forces.x/CP; // we can multipe a number to rescale eta
    }
    String Sp = SparsePressure(test);

    //println("====[debug]===="+str(forces.x)+str(CP));
    String state_cn = multy_state(y,angle,velocitx,velocity,velocityphi,Eta,Sp);
    state.clear();
    state.add(state_cn);
    
    
    //println("[debug]AoA= " + AoA + " " + "A= " + A + " " + "y= " + test.foil.xc.y + " " + "theta= " + test.foil.phi * 180 / PI);
    //println("[debug][state]:"+state);
    
    reward(CT);
    output.println("" + AoA + "," + A + "," + CT);

    
    //TODO: why clear here?
    //state.clear();


    dat.output.println(test.t + " " + forces.x + " " + forces.y + " " + angle + " " + test.body.bodyList.get(0).xc.y  + " " + velocitx + " " + velocity + " " + velocityphi + " " +CT + " " + CP + " " + Eta + " " + Sp + ";");
    CT = 0;
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
    //dat.finish();
    //output.close();
    //EpisodeNum += 1;
    //setUpNewSim(EpisodeNum);
    test.t = 0;
  }
}

// remote call action
float[] callActionByRemote()
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
  //+ "action2:"+ String.valueOf(action[2])+ "action3:"+ String.valueOf(action[3]));
  return action;
}

/*
//PVT motion interpolation
float [] PVTMode(float[] NA, float []FA, float StepTime, float CurrentStep){
  // Smooth parameters 
  float Ab0 = FA[0];
  float Ab1 = FA[1];
  float Ab2 = 3*(NA[0]-FA[0])/pow(StepTime,2) - (2*FA[1]+NA[1])/StepTime;
  float Ab3 = (NA[1]+FA[1])/pow(StepTime,2)+2*(FA[0]-NA[0])/pow(StepTime,3);
  float AOAb0 = FA[2];
  float AOAb1 = FA[3];
  float AOAb2 = 3*(NA[2]-FA[2])/pow(StepTime,2) - (2*FA[3]+NA[3])/StepTime;
  float AOAb3 = (NA[3]+FA[3])/pow(StepTime,2)+2*(FA[2]-NA[2])/pow(StepTime,3);
  // Smoothed single step motion
  float smoothA = Ab0+Ab1*CurrentStep+Ab2*pow(CurrentStep,2)+Ab3*pow(CurrentStep,3);
  float smoothAvel = Ab1+2*Ab2*CurrentStep+3*Ab3*pow(CurrentStep,2);
  float smoothAoA = AOAb0+AOAb1*CurrentStep+AOAb2*pow(CurrentStep,2)+AOAb3*pow(CurrentStep,3);
  float smoothAoAvel = AOAb1+2*AOAb2*CurrentStep+3*AOAb3*pow(CurrentStep,2);
  // Array state
  float smoothedState[] = {smoothA,smoothAvel,smoothAoA,smoothAoAvel};
  return smoothedState;
}
*/

void reward(float CT)
{
  float target_reward = 0;
  if (abs(CT) > 100)
  {
    target_reward = -300;
    //CT = CT/abs(CT) * 100;
    done = true;
  }
  if(Double.isNaN(CT))
  {
    println("[Warning]: env crashed!");
    target_reward = -1000;
    done = true;
  }
  // TODO: !!! update reward , done and state in buffer
  reward_buffer = target_reward;
  //release state semaphore to let server return the resulted state
  state_sem.release();
}
// start a server
WebServer StartServer(int port)
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
        //TODO: Add vel to action
        action[0] = input_object.getFloat("yvel");
        action[1] = input_object.getFloat("thetavel");
        action[2] = 0.0;//input_object.getFloat("xvel");
        //action[2] = input_object.getFloat("theta");
        //action[3] = input_object.getFloat("thetavel");
        
        // release action, and then
        action_sem.release();
        //println("[debug]action:", action[0], "  ", action[1]);//, "  ",action[2], "  ", action[3]);

        // query new state, reward , done and wait
        //TODO: maybe some bug
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
public String multy_state(float py,float ptheta,float yvel,float xvel,float tvel,float peta,String Sp) {
  JSONObject multy_state_json = new JSONObject();
  multy_state_json.put("delta_y", py);
  multy_state_json.put("delta_theta", ptheta);
  multy_state_json.put("y_velocity", yvel);
  multy_state_json.put("x_velocity", xvel);
  multy_state_json.put("theta_velocity", tvel);
  multy_state_json.put("eta", peta);
  multy_state_json.put("SparsePressure", Sp);
  // CT and other info for average CT and eta
  multy_state_json.put("ct", CT);
  multy_state_json.put("cp", CP);
  multy_state_json.put("force_x", forceX);
  multy_state_json.put("dt", test.flow.dt/32.);
  println(multy_state_json);

 return multy_state_json.toJSONString();
}

void
setUpNewSim(int runNum)
{
  new File(datapath + "Episode" + str(runNum)).mkdir();
  test = new FoilUnsteadywake(resolution, xLengths, yLengths, tStep, Re, true, true);
  dat = new SaveData(datapath + "Episode" + str(runNum) + "/force.txt", test.body.bodyList.get(0).coords, resolution, xLengths, yLengths, zoom);
  dat.output.println("t" + " " + "fx" + " " + "fy" + " " + "theta" + " " + "y" + " " + "xvel"+ " " + "yvel"+ " " + "tvel"+ " " + "ct"+ " " + "cp+ " + " "  + "eta"+" "+"SP");
  output = createWriter(datapath + "Episode" + str(runNum) + "/output.csv");
  output.println("" + "Action1" + "," + "Action2" + "," + "CT");
  println("Episode" + str(runNum) + " start!");

  AoA=0;
  nextactionAoA=0;
  A=0;
  nextactionA=0;
}
//SparsePressure
String SparsePressure(FoilUnsteadywake test)
{
  float[] Pressure = new float[0];
  for (int i = 0; i < (test.body.bodyList.get(0).coords.size()) / 10; i++)
  {
    Pressure = append(Pressure, test.flow.p.linear(test.body.bodyList.get(0).coords.get(i * 10).x, test.body.bodyList.get(0).coords.get(i * 10).y));
  }
  //Pressure = append(Pressure, test.flow.p.linear(test.body.bodyList.get(0).coords.get(199).x, test.body.bodyList.get(0).coords.get(199).y));
  p = String.valueOf(Pressure[0]);
  for (int i = 1; i < Pressure.length; i++)
  {
    p = p + "_" + String.valueOf(Pressure[i]);
  }
  return p;
}
float[] ycoords(FoilUnsteadywake test)
{
  float[] ycoords = new float[0];
  for (int i = 0; i < (test.body.bodyList.get(0).coords.size()); i++)
  {
    ycoords = append(ycoords, test.body.bodyList.get(0).coords.get(i).y);
  }
  return ycoords;
}
//.
int sign(float x)
{
  int s = 0;
  if (x > 0)
  {
    s = 1;
  }
  if (x < 0)
  {
    s = -1;
  }
  if (x == 0)
  {
    s = 0;
  }
  return s;
}
