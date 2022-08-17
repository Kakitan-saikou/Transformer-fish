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
 
    joint1 = new Part_head(1.*resolution,2.*resolution,1.*resolution,0.12,window);
    joint2 = new Part(2.*resolution,2.*resolution,1.*resolution,0.12,window);
    joint3 = new Part_tail(3.*resolution,2.*resolution,1.*resolution,0.12,window);
    //joint2.rotate_init(Max_theta*sin(Delta_theta));
    fish3 = new BodyUnion(joint1, joint2);
    fish3.add(joint3);
    flow = new BDIM(2*n,n,0,fish3,(float)resolution_test/Re_test,true);
    //int PInumberForOutput = int(PI/Max_theta);

    
    flood = new FloodPlot(window);
    flood.range = new Scale(-.5,.5);
    flood.setLegend("vorticity");
    flood.setColorMode(1);
  }
  /*Fish3( int resolution, int xStart, float zoom, int Re) {
    this(resolution, 100, 50, xStart, zoom, Re, 1);
    window = new Window(1000, 500);
  }*/
  
  
  

//修改了updatePosition--增加了action接口
  void updatePosition(float dt, float omega1, float omega2, float omega3, float omega4, int flag){
    
    if(flag==0){
      print("");
    }
    else if(flag==1){
      if(time>=0){
        // joint1.rotate_xRandom(omega_c(.2, 0.2, PI/3*2)*flow.dt, PVector.mult(PVector.add(joint1.coords.get(75),joint1.coords.get(125)),0.5));
        // joint1.rotate_toPhi_xRandom(omega1,
              // PVector.mult(PVector.add(joint1.coords.get(75),joint1.coords.get(125)),0.5));
        joint1.rotate_toPhi_xRandom(omega1,
              joint1.coords.get(101));
        joint1.translate(0, (resolution * omega4) / 50);
        // joint1.rotate_xRandom(omega1, PVector.mult(PVector.add(joint1.coords.get(75),joint1.coords.get(125)),0.5));
        // connection(joint2, omega_c(.2, 0.3, 0), joint1);//rotate_dalpha, theta0, alpha0
        // connection(joint3, omega_c(-.2, 0.3, 0), joint2);
        connection(joint2, omega2, joint1, 1);//rotate_dalpha, theta0, alpha0
        connection(joint3, omega3, joint2, 1);
        // println(omega, scale);
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
  void update(float omega1, float omega2, float omega3, float omega4) {
    if (flow.QUICK) {
      dt = flow.checkCFL();
      flow.dt = dt;//modify
    }
    updatePosition(dt,omega1,omega2,omega3,omega4,1);//修改
    flow.update(fish3);flow.update2();
    t += dt;
  }
  void display() {
    flood.display(flow.u.curl());
    fish3.display();
  }
  void connection(Body joint, float omega, float vx, float vy){
    // joint.rotate(omega*flow.dt);
    // 修改为转的角度
    joint.rotate(omega);
    joint.translate((-sin(joint.phi)*resolution*0.25*omega+vx)*flow.dt, 
                    (cos(joint.phi)*resolution*0.25*omega+vy)*flow.dt);
    //joint.rotate(omega3*flow.dt);
  }
    void connection(Body joint, float phi_new, float vx, float vy, float flag_phi){///phi
    joint.rotate_toPhi(phi_new);
    joint.translate((cos(phi_new)-cos(joint.phi))*resolution*0.25+vx*flow.dt,
                    (sin(phi_new)-sin(joint.phi))*resolution*0.25+vy*flow.dt);
                    //(-sin(joint.phi)*resolution*0.25*(phi_new-joint.phi)/flow.dt+vx)*flow.dt, 
                    //(cos(joint.phi)*resolution*0.25*(phi_new-joint.phi)/flow.dt+vy)*flow.dt);
    //joint.rotate(omega3*flow.dt);
  }
  void connection(Body joint,float omega){
    connection(joint, omega,0,0);
  }
  void connection(Body joint,float omega, Body joint0){
    float vx=joint0.velocity(1,dt,joint0.coords.get(100).x,joint0.coords.get(100).y);
    float vy=joint0.velocity(2,dt,joint0.coords.get(100).x,joint0.coords.get(100).y);
    connection(joint, omega, vx, vy);
    float delta_x = joint0.coords.get(100).x - joint.coords.get(1).x;
    float delta_y = joint0.coords.get(100).y - joint.coords.get(1).y;
    //println(delta_x,delta_y);
    joint.translate(delta_x,delta_y);
  }
  void connection(Body joint,float phi_new, Body joint0, float flag_phi){///phi
    float vx=joint0.velocity(1,dt,joint0.coords.get(100).x,joint0.coords.get(100).y);
    float vy=joint0.velocity(2,dt,joint0.coords.get(100).x,joint0.coords.get(100).y);
    connection(joint, phi_new, vx, vy,1);
    float delta_x = joint0.coords.get(100).x - joint.coords.get(1).x;
    float delta_y = joint0.coords.get(100).y - joint.coords.get(1).y;
    //println(delta_x,delta_y);
    joint.translate(delta_x,delta_y);
  }
  float omega_c (float rotate_dalpha, float theta0, float alpha0){
    return theta0*rotate_dalpha*cos(rotate_dalpha*time+alpha0);
  }
  float omega_c (float rotate_dalpha, float theta0){
    return omega_c(rotate_dalpha, theta0, 0);
  }
  float theta_c (float rotate_dalpha, float theta0, float alpha0){
    return theta0*sin(rotate_dalpha*time+alpha0);
  }
}