import java.util.*;
import org.apache.xmlrpc.*;
import java.util.concurrent.*;
import java.util.concurrent.Semaphore;
import com.alibaba.fastjson.JSONObject;
RotatingFoil test;
SaveData dat;
float maxT;
PrintWriter output;

int Re = 1173;
float chord = 1.0;
//float dAoA = 25*PI/180, uAoA = dAoA;
int resolution = 32, xLengths = 10, yLengths =12, zoom = 3;
int picNum = 10;
float tStep = .005;
String datapath = "saved" + "/";
int EpisodeNum = 1;
int EpisodeTime = 50, ActionTime = 0; //TODO: actionTime = 5
float CT = 0, CL = 0, CP = 0;
float AoA = 0, A = 160;
float Eta = 0;
float y = 0, angle = 0;
float vy =0, vx = 0;
float nextactionAoA=0,nextactionA=0;
String p;
XmlRpcClient client;
WebServer server;
float[] ycoords;
int calllearn = 1,Num_steps=3;

//remote info exchange
ArrayList state = new ArrayList();
float[] action = new float[2];
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
  println(args);


  try
  {

    server = StartServer(int(args[0]));


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
     setUpNewSim(EpisodeNum);
     test.t = 0;
     done = false;
     reset_flag = 0;
     ArrayList state_cn = multy_state(y,angle,0,0,0,Eta);
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
      test.update2(AoA, A);
      test.display();
      PVector forces = test.foil.pressForce(test.flow.p);
      CT += -forces.x / resolution * 2;
      ycoords = ycoords(test);
      angle = (test.foil.phi - PI) / PI * 180.;
      PVector coordinate = test.foil.xc;
      y = coordinate.y - yLengths * resolution / 2.;
      PVector velocity = test.foil.dxc;
      float dphi = test.foil.dphi/test.flow.dt;
      float dy = velocity.y/test.flow.dt;
      vy = velocity.y;
      vx = velocity.x;
      float M = test.foil.pressMoment(test.flow.p);
      CP  = (M*dphi+forces.y*dy)/resolution*test.flow.dt/resolution;
      Eta  = CT/CP*100;
      ArrayList state_cn = multy_state(y,angle,vx,vy,0,Eta);
      state.clear();
      state.add(state_cn);
    }
    else if(test.t>=ActionTime){
    // TODO: read action from server logger
    //System.out.println("[debug]before action...");
    float[] NextAction = callActionByRemote();
    //System.out.println("[debug]after action...");
    //Action limitation for pitch and heave
    nextactionAoA = NextAction[0];
    nextactionA = NextAction[1];
    // Add limitation to the python output command
    // Each action A<1.2, AoA<1.5; (A and AoA are relative position change value)
    A = A + nextactionA; // 20-300 heave limit
    AoA = AoA + nextactionAoA; // 70 degree pitch limit
    if (A>=250||A<=50){
      A = A - 2*nextactionA;
    }
    if (abs(AoA)>=70){
      AoA = AoA - 2*nextactionAoA;
    }

    test.update2(AoA, A);
    test.display();
    PVector forces = test.foil.pressForce(test.flow.p);
    ycoords = ycoords(test);
    angle = (test.foil.phi - PI) / PI * 180.;
    PVector coordinate = test.foil.xc;
    y = coordinate.y - yLengths * resolution / 2.;
    PVector velocity = test.foil.dxc;
    float dphi = test.foil.dphi/test.flow.dt;
    float dy = velocity.y/test.flow.dt;
    float M = test.foil.pressMoment(test.flow.p);
    vy = velocity.y;
    vx = velocity.x;
    CP  = (M*dphi+forces.y*dy)/resolution*test.flow.dt/resolution;
    Eta  = CT/CP*100;
    ArrayList state_cn = multy_state(y,angle,vx,vy,0,Eta);
    state.clear();
    state.add(state_cn);

    CT += -forces.x / resolution * 2;
    //println("AoA= " + AoA + " " + "A= " + A + " " + "y= " + test.foil.xc.y + " " + "theta= " + test.foil.phi * 180 / PI);
    //println(state);
    reward(CT);
    output.println("" + AoA + "," + A + "," + CT);
    CT = 0;

    //TODO: why clear here?
    //state.clear();


    dat.output.println(test.t + " " + forces.x + " " + forces.y + " " + angle + " " + test.foil.xc.y + ";");
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
  return action;
}

void reward(float CT)
{
  if (abs(CT) > 100)
  {
    CT = -300;
    //CT = CT/abs(CT) * 100;
    done = true;
  }
  if(Double.isNaN(CT))
  {
    CT = -1000;
    done = true;
  }
  // TODO: !!! update reward , done and state in buffer
  reward_buffer = CT;
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
        action[0] = input_object.getFloat("y");
        action[1] = input_object.getFloat("theta");
        // release action, and then
        action_sem.release();
        //println("action:", action[0], "  ", action[1]);

        // query new state, reward , done and wait
        //TODO: maybe some bug
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
  list.add(pfx);list.add(pfy);
  //list.add(0);list.add(0);
  if(Double.isNaN(psp)){
  psp = 0;
  }
  list.add(peta);list.add(psp);
 return list;
}

void
setUpNewSim(int runNum)
{
  new File(datapath + "Episode" + str(runNum)).mkdir();
  test = new RotatingFoil(resolution, xLengths, yLengths, tStep, Re, true, true);
  dat = new SaveData(datapath + "Episode" + str(runNum) + "/force.txt", test.foil.coords, resolution, xLengths, yLengths, zoom);
  dat.output.println("t" + " " + "fx" + " " + "fy" + " " + "theta" + " " + "y");
  output = createWriter(datapath + "Episode" + str(runNum) + "/output.csv");
  output.println("" + "Action1" + "," + "Action2" + "," + "CT");
  println("Episode" + str(runNum) + " start!");

  AoA=0;
  nextactionAoA=0;
  A=80;
  nextactionA=0;//foil fang dao zhong jian
}
//SparsePressure
String SparsePressure(RotatingFoil test)
{
  float[] Pressure = new float[0];
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
float[] ycoords(RotatingFoil test)
{
  float[] ycoords = new float[0];
  for (int i = 0; i < (test.foil.coords.size()); i++)
  {
    ycoords = append(ycoords, test.foil.coords.get(i).y);
  }
  return ycoords;
}

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
