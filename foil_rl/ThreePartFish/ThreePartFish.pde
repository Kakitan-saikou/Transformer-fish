//Added by XYC
//Added here-1
import java.util.*;
import org.apache.xmlrpc.*;
import java.util.concurrent.*;
import java.util.concurrent.Semaphore;
import com.alibaba.fastjson.JSONObject;
//over-1

//FishRL
Fish3 test;
BDIM flow;
//Bread fish;
FloodPlot flood;
float time=0;
int resolution = 16;
float Re_test = 2000;//Re=6000
float St_test = 0.5;//Str=0.45

// Added here-2
SaveData dat;
float maxT;
PrintWriter output;
//over-2

//state 是pressforce什么的吗


// void setup(){
//   size(1600, 800); 
//   test = new Fish3(resolution, Re_test, St_test, true);
// }
// void draw(){
//   test.update();
//   test.display();
  
// }

// Added here-3
int picNum = 10;
float tStep = .005;
String datapath = "saved" + "/";
int EpisodeNum = 1;
int EpisodeTime = 2000, ActionTime = 0; //TODO: actionTime = 5
float CT = 0, CL = 0, CP = 0;
// float AoA = 0, A = 160;
float A1 = 0, A2 = 0, A3 = 0, A4 = 0, A5 = 0;
float Eta = 0;
float niuju = 0;
float eta_fish3 = 0;
float a_r = 0;
float d_r = 0;
float y_r = 0;
float t_r = 0;
float stepr = 0;
float dt = 0;
float penalty = 0;
float omega_r = 0;
float angle0 = 0, angle1 = 0, angle2 = 0;
float jiao0 = 0, jiao1 = 0, jiao2 = 0;
float zuobiaoy = 0;
float angle_2 = 0;
float dd0 = 0, dd1=0, dd2=0;
float omega0 = 0, omega1 = 0, omega2 = 0;
float press1 = 0, press1_mid = 0, press1_mid_d = 0, press1_tail = 0, press1_tail_d = 0, press2_mid = 0, press2_mid_d = 0, press2_tail = 0, press2_tail_d = 0, press3_mid = 0, press3_mid_d = 0, press3_tail = 0, press3_tail_d = 0;
float fishfx = 0, fishfy = 0;
float Ix_dt = 0, Iy_dt = 0;
float W1_dt = 0, W2_dt = 0, W3_dt = 0, W = 0;
float nextactionAoA=0,nextactionA=0;
float done_flag = 0;
String p;
XmlRpcClient client;
WebServer server;
// float[] ycoords;
int calllearn = 1,Num_steps=3;

//remote info exchange
ArrayList state = new ArrayList();
float[] action = new float[5];  //先令action size=3
float reward_buffer = 0;
Boolean done = false;
int reset_flag = 0;
// Semaphore to provide asynchronization
Semaphore action_sem = new Semaphore(0);
Semaphore state_sem = new Semaphore(0);

void setup()
{
  size(1600, 800); 
  //surface.setVisible(false);
  println(args);

  try
  {

    server = StartServer(int(args(0));

  }
  catch (Exception ex)
  {
    println(ex);
  }
  setUpNewSim(EpisodeNum);
}

void draw(){
  if (test.t <= EpisodeTime){
    if(reset_flag == 1){
    //  dat.finish();
    //  output.close();
     EpisodeNum +=1;
    //  A = 0;
    //  AoA = 80;
     A1 = 0;
     A2 = 0;
     A3 = 0;
     A4 = 0;
     A5 = 0;
     setUpNewSim(EpisodeNum);
     test.t = 0;
     done = false;
     reset_flag = 0;
     done_flag = 0;

// TODO 这里要加哪些state，以及state的计算方式待修改
    //  ArrayList state_cn = multy_state(angle1,angle2,omega1,omega2,dd1,dd2,fishfx,fishfy,CT);
      ArrayList state_cn = multy_state(jiao0,jiao1,jiao2,omega0,omega1,omega2,zuobiaoy,
      press1, press1_mid, press1_mid_d, press1_tail, press1_tail_d, press2_mid, press2_mid_d,
      press2_tail, press2_tail_d, press3_mid, press3_mid_d, press3_tail, press3_tail_d);
  
     state.clear();
     state.add(state_cn);
    }
    if(done == true){// done == true stop sim
        // continue
        // dat.finish();
        // output.close();

    }
    if (test.t<ActionTime){
      // A = 80;
      // AoA = 0;
      // test.update2(AoA, A);
      A1 = 0;
      A2 = 0;
      A3 = 0;
      A4 = 0;
      A5 = 0;
      test.update(A1, A2, A3, A4);
      // test.update(A1, A2);
      test.display();
//TODO foil环境里是通过NACA foil的pressforce计算的力，但是fish3里没有对应的力的计算

//TODO 计算角度，关于是哪个joint的角度我也不知道，所以就直接设为2和3的了
    jiao0 = test.joint1.phi;
    jiao1 = test.joint2.phi;
    jiao2 = jiao1 - test.joint3.phi;
    angle0 = jiao0 / PI * 180.;
    angle1 = jiao1 / PI * 180.;
    // angle_2 = test.joint3.phi / PI * 180.;
    // // angle2 = -angle1 + 180 + angle_2;
    angle2 = jiao2 / PI * 180.;
    omega0 = test.joint1.dphi;
    omega1 = test.joint2.dphi;
    omega2 = test.joint3.dphi;
    dd0 = test.joint1.ddotphi;
    dd1 = test.joint2.ddotphi;
    dd2 = test.joint3.ddotphi;
    zuobiaoy = (test.joint1.coords.get(100).y - 31) / 10;
    dt = test.flow.checkCFL();

    OrthoNormal otest1=test.joint1.orth[1];
    OrthoNormal otest1_mid=test.joint1.orth[51];
    OrthoNormal otest1_mid_d=test.joint1.orth[151];
    OrthoNormal otest1_tail=test.joint1.orth[98];
    OrthoNormal otest1_tail_d=test.joint1.orth[104];

    OrthoNormal otest2_mid=test.joint2.orth[51];
    OrthoNormal otest2_mid_d=test.joint2.orth[151];
    OrthoNormal otest2_tail=test.joint2.orth[98];
    OrthoNormal otest2_tail_d=test.joint2.orth[104];

    OrthoNormal otest3_mid=test.joint3.orth[51];
    OrthoNormal otest3_mid_d=test.joint3.orth[151];
    OrthoNormal otest3_tail=test.joint3.orth[98];
    OrthoNormal otest3_tail_d=test.joint3.orth[104];

    press1 = test.flow.p.linear(otest1.cen.x, otest1.cen.y);
    press1_mid = test.flow.p.linear(otest1_mid.cen.x, otest1_mid.cen.y);
    press1_mid_d = test.flow.p.linear(otest1_mid_d.cen.x, otest1_mid_d.cen.y);
    press1_tail = test.flow.p.linear(otest1_tail.cen.x, otest1_tail.cen.y);
    press1_tail_d = test.flow.p.linear(otest1_tail_d.cen.x, otest1_tail_d.cen.y);

    press2_mid = test.flow.p.linear(otest2_mid.cen.x, otest2_mid.cen.y);
    press2_mid_d = test.flow.p.linear(otest2_mid_d.cen.x, otest2_mid_d.cen.y);
    press2_tail = test.flow.p.linear(otest2_tail.cen.x, otest2_tail.cen.y);
    press2_tail_d = test.flow.p.linear(otest2_tail_d.cen.x, otest2_tail_d.cen.y);

    press3_mid = test.flow.p.linear(otest3_mid.cen.x, otest3_mid.cen.y);
    press3_mid_d = test.flow.p.linear(otest3_mid_d.cen.x, otest3_mid_d.cen.y);
    press3_tail = test.flow.p.linear(otest3_tail.cen.x, otest3_tail.cen.y);
    press3_tail_d = test.flow.p.linear(otest3_tail_d.cen.x, otest3_tail_d.cen.y);
    
    println("zuobiaoysj"+str(zuobiaoy));
    fishfy = test.joint1.pressForce(test.flow.p).y + test.joint2.pressForce(test.flow.p).y
           + test.joint3.pressForce(test.flow.p).y;
    fishfx = test.joint1.pressForce(test.flow.p).x + test.joint2.pressForce(test.flow.p).x 
           + test.joint3.pressForce(test.flow.p).x;
    CT = -fishfx / resolution * 2;
//TODO: 没有相关的计算位置与速度的接口！fish3里看起来只有角速度相关的？

      ArrayList state_cn = multy_state(jiao0,jiao1,jiao2,omega0,omega1,omega2,zuobiaoy,
      press1, press1_mid, press1_mid_d, press1_tail, press1_tail_d, press2_mid, press2_mid_d,
      press2_tail, press2_tail_d, press3_mid, press3_mid_d, press3_tail, press3_tail_d);

      state.clear();
      state.add(state_cn);
    }
    else if(test.t>=ActionTime){
    //read action from server logger
    //System.out.println("[debug]before action...");
    float[] NextAction = callActionByRemote();

    // A1 = NextAction[0] * test.flow.checkCFL() + A1;
    // A2 = NextAction[1] * test.flow.checkCFL() + A2;
    // A1 = NextAction[0] + A1;
    // A2 = NextAction[1] + A2;
    // A3 = NextAction[2] + A3;

    // A0 = NextAction[0];
    A1 = NextAction[0];
    A2 = NextAction[1];
    A3 = NextAction[2];
    A4 = NextAction[3];
    // A5 = NextAction[4];
    // println("Actions " + NextAction[0] + "_" + NextAction[1]);

    test.update(A1, A2, A3, A4);
    // test.update(A0, A1, A2);
    test.display();

//TODO: 和上面一样的问题，state什么的

    jiao0 = test.joint1.phi;
    jiao1 = test.joint2.phi;
    jiao2 = jiao1 - test.joint3.phi;
    angle0 = jiao0 / PI * 180.;
    angle1 = jiao1 / PI * 180.;
    // angle_2 = test.joint3.phi / PI * 180.;
    // // angle2 = -angle1 + 180 + angle_2;
    angle2 = jiao2 / PI * 180.;
    omega0 = test.joint1.dphi;
    omega1 = test.joint2.dphi;
    omega2 = test.joint3.dphi;
    dd0 = test.joint1.ddotphi;
    dd1 = test.joint2.ddotphi;
    dd2 = test.joint3.ddotphi;
    zuobiaoy = (test.joint1.coords.get(100).y - 31) / 10;
    dt = test.flow.checkCFL();

    OrthoNormal otest1=test.joint1.orth[1];
    OrthoNormal otest1_mid=test.joint1.orth[51];
    OrthoNormal otest1_mid_d=test.joint1.orth[151];
    OrthoNormal otest1_tail=test.joint1.orth[98];
    OrthoNormal otest1_tail_d=test.joint1.orth[104];

    OrthoNormal otest2_mid=test.joint2.orth[51];
    OrthoNormal otest2_mid_d=test.joint2.orth[151];
    OrthoNormal otest2_tail=test.joint2.orth[98];
    OrthoNormal otest2_tail_d=test.joint2.orth[104];

    OrthoNormal otest3_mid=test.joint3.orth[51];
    OrthoNormal otest3_mid_d=test.joint3.orth[151];
    OrthoNormal otest3_tail=test.joint3.orth[98];
    OrthoNormal otest3_tail_d=test.joint3.orth[104];

    press1 = test.flow.p.linear(otest1.cen.x, otest1.cen.y);
    press1_mid = test.flow.p.linear(otest1_mid.cen.x, otest1_mid.cen.y);
    press1_mid_d = test.flow.p.linear(otest1_mid_d.cen.x, otest1_mid_d.cen.y);
    press1_tail = test.flow.p.linear(otest1_tail.cen.x, otest1_tail.cen.y);
    press1_tail_d = test.flow.p.linear(otest1_tail_d.cen.x, otest1_tail_d.cen.y);

    press2_mid = test.flow.p.linear(otest2_mid.cen.x, otest2_mid.cen.y);
    press2_mid_d = test.flow.p.linear(otest2_mid_d.cen.x, otest2_mid_d.cen.y);
    press2_tail = test.flow.p.linear(otest2_tail.cen.x, otest2_tail.cen.y);
    press2_tail_d = test.flow.p.linear(otest2_tail_d.cen.x, otest2_tail_d.cen.y);

    press3_mid = test.flow.p.linear(otest3_mid.cen.x, otest3_mid.cen.y);
    press3_mid_d = test.flow.p.linear(otest3_mid_d.cen.x, otest3_mid_d.cen.y);
    press3_tail = test.flow.p.linear(otest3_tail.cen.x, otest3_tail.cen.y);
    press3_tail_d = test.flow.p.linear(otest3_tail_d.cen.x, otest3_tail_d.cen.y);

    fishfy = test.joint1.pressForce(test.flow.p).y + test.joint2.pressForce(test.flow.p).y
           + test.joint3.pressForce(test.flow.p).y;
    fishfx = test.joint1.pressForce(test.flow.p).x + test.joint2.pressForce(test.flow.p).x 
           + test.joint3.pressForce(test.flow.p).x;
    CT = -fishfx / resolution * 2;

    //拆开三个关节的fishfy domega

    // println(angle1, angle2);

    // ArrayList state_cn = multy_state(jiao0,jiao1,jiao2,omega0,omega1,omega2,fishfx,fishfy);
      ArrayList state_cn = multy_state(jiao0,jiao1,jiao2,omega0,omega1,omega2,zuobiaoy,
      press1, press1_mid, press1_mid_d, press1_tail, press1_tail_d, press2_mid, press2_mid_d,
      press2_tail, press2_tail_d, press3_mid, press3_mid_d, press3_tail, press3_tail_d);

    state.clear();
    state.add(state_cn);
    // println("States-Angles " + str(angle1) + " "+ str(angle2));
    // println("States " + str(omega1) + "_" + str(omega2));

    // Ix_dt = ( test.joint1.pressForce(test.flow.p).x + test.joint2.pressForce(test.flow.p).x 
    //       + test.joint3.pressForce(test.flow.p).x ) * test.flow.checkCFL();
    // Iy_dt = ( test.joint1.pressForce(test.flow.p).y + test.joint2.pressForce(test.flow.p).y 
    //       + test.joint3.pressForce(test.flow.p).y ) * test.flow.checkCFL();
    Ix_dt = fishfx * dt;
    Iy_dt = abs(fishfy * dt);
    W1_dt = (test.joint1.pressForce(test.flow.p).x * test.joint1.dxc.x) 
          + (test.joint1.pressForce(test.flow.p).y * test.joint1.dxc.y)
          + (test.joint1.pressMoment(test.flow.p) * test.joint1.dphi);
    W2_dt = (test.joint2.pressForce(test.flow.p).x * test.joint2.dxc.x) 
          + (test.joint2.pressForce(test.flow.p).y * test.joint2.dxc.y)
          + (test.joint2.pressMoment(test.flow.p) * test.joint2.dphi);
    W3_dt = (test.joint3.pressForce(test.flow.p).x * test.joint3.dxc.x) 
          + (test.joint3.pressForce(test.flow.p).y * test.joint3.dxc.y)
          + (test.joint3.pressMoment(test.flow.p) * test.joint3.dphi);
    // eta_fish3 = Ix_dt - (W1_dt + W2_dt + W3_dt);
    // println("W" + str(W2_dt + W3_dt));
    // if (abs(W1_dt) > 0){println("Wrong W1" + str(W1_dt));}
    W = W1_dt + W2_dt + W3_dt;
    // niuju = test.joint1.pressMoment(test.flow.p) + test.joint2.pressMoment(test.flow.p) + test.joint3.pressMoment(test.flow.p);

    // a_r = - exp(abs(angle1) - 20) - exp(abs(angle1) - 30) - exp(abs(angle2) - 55);
    a_r = - abs(max(abs(angle0), 20)) - abs(max(abs(angle1), 20)) - abs(max(abs(angle2), 55)) + 95;
    d_r = - abs(max(abs(dd0), 0.05)) - abs(max(abs(dd1), 0.05)) - abs(max(abs(dd2), 0.05)) + 0.15;
    y_r = - abs(max(abs(zuobiaoy), 5)) + 5;
    t_r = dt - 0.65;
    // omega_r = - exp((abs(angle1) - 30) * abs(omega1)) - exp((abs(angle2) - 60) * abs(omega2)); //关于角度与omega的联动关系
    // y_r = - exp(abs(test.joint1.coords.get(100).y - 2.*resolution) - 10);  //关于y方向上位移的惩罚
    println("dt:" + str(dt) + "ar:" + str(a_r) + "dr:" + str(d_r) + "yr:" + str(y_r));
    println("Ix:" + str(Ix_dt) + "Iy:" + str(Iy_dt) + "W:" + str(W));

    // y_r = test.joint1.coords.get(101).y - 2.*resolution;
    // println("yr:" + str(y_r));
    // println("eta" + str(eta_fish3));
    // println("angle_penalty" + str(a_r));、
    penalty = a_r + d_r * 10 + y_r + t_r * 3;
    // stepr = penalty + 80 * Ix_dt - Iy_dt - 1.5 * W;
    stepr = Ix_dt;


    reward(stepr, angle1, angle2, dt);
    stepr = 0;


    // dat.output.println(test.t + " " + angle1 + " " + angle2 + ";");
    //println("EpisodeNUm=" + EpisodeNum);
    // if (test.t > 40)
    //   {
    //     picNum--;
    //     if (picNum <= 0)
    //     {
    //       saveFrame(datapath + "Episode" + str(EpisodeNum) + "/" + "frame-#######.png");
    //       picNum = 10;
    //     }
    //   }
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

//TODO 写reward函数
void reward(float stepr, float angle1, float angle2, float dt)
{
  // TODO: !!! update reward , done and state in buffer
  //You may need to claim the variables(all float) in the head of the code
  //给domega一个线性负reward
  //更新terminate条件
  float pan = abs(angle1);
  float pan2 = angle1;
  float pan3 = angle1 - angle2;

  // float pan3 = abs(dd1);
  // float pan4 = abs(dd2);
  if (pan > 45)
  {
    stepr = stepr - 500;
    if (pan > 60)
    {
      stepr = stepr - 500;
      // done_flag += 1;
      done = true;
    }
  }

  if (pan2 > 0){
    if (pan3 > 45 || pan3 < -60){
      stepr = stepr - 500;
      // done_flag += 1;
      done = true;
    }
  }
  else{
    if (pan3 > 60 || pan3 < -45){
      stepr = stepr - 500;
      // done_flag += 1;
      done = true;
    }
  }

  if (dt < 0.05){
    stepr = stepr - 500;
    done = true;
  }


  // println("Step Reward " + str(stepr));
  
  reward_buffer = stepr;
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
        action[0] = input_object.getFloat("a1");
        action[1] = input_object.getFloat("a2");
        action[2] = input_object.getFloat("a3");
        action[3] = input_object.getFloat("a4");
        // action[4] = input_object.getFloat("a5");
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
        // output_object.put("state", state);
        output_object.put("state", state);
        output_object.put("reward", reward_buffer);
        output_object.put("done", done);
        // output_object.put("dI", Ix_dt);
        // output_object.put("dIy", Iy_dt);
        // output_object.put("dW", W);


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
public ArrayList multy_state(float py,float ptheta,float pfx,float pfy, float peta, float psp, float j2, float p1, float p2,
float p3, float p4, float p5, float p6, float p7, float p8, float p9, float p10, float p11, float p12, float p13) {
  ArrayList list = new ArrayList();
  list.add(py);list.add(ptheta);
  // list.add(pfx);list.add(pfy);
  list.add(pfx);list.add(pfy);
  list.add(peta);list.add(psp);
  // list.add(CT);list.add(j1);
  list.add(j2);
  list.add(p1); list.add(p2); list.add(p3); list.add(p4); list.add(p5); list.add(p6); list.add(p7); list.add(p8);
  list.add(p9); list.add(p10); list.add(p11); list.add(p12); list.add(p13);
 return list;
}

void
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

  A1 = 0;
  A2 = 0;
  A3 = 0;
  A4 = 0;
  A5 = 0;
}
//over-3
