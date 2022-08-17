float invSqrt(float x) {//fast inv sqrt; f=1/x^(1/2)
  float xhalf = 0.5f * x;
  int i = Float.floatToIntBits(x);
  i = 0x5f3759df - (i >> 1);
  x = Float.intBitsToFloat(i);
  x *= (1.5f - xhalf * x * x);
  x *= (1.5f - xhalf * x * x);//can be deleted for quicker calculation
  return x;
}


float P_x(Body body, Field p, int i){ 
  return (body.orth[i].ty)*p.a[(int)body.coords.get(i).x][(int)body.coords.get(i).y];
}
float Fx_Sum(Body body, Field p){
  float Psum = 0;
  for (int i=0; i<body.n; i++){
    Psum += P_x(body, p, i)*body.orth[i].l;
    //println("Px= "+P_x(body, p, i)+" Py= "+body.orth[i].l);
  }
  return Psum;
}
float P_y(Body body, Field p, int i){ 
  return (body.orth[i].tx)*p.a[(int)body.coords.get(i).x][(int)body.coords.get(i).y];
}
float Fy_Sum(Body body, Field p){
  float Psum = 0;
  for (int i=0; i<body.n; i++){
    Psum += P_y(body, p, i)*body.orth[i].l;
    //println("Px= "+P_x(body, p, i)+" Py= "+body.orth[i].l);
  }
  return Psum;
}
float M(Body body,Field p, int i){
  float M_x = P_x(body,p,i)*body.orth[i].l*(body.coords.get(i).y-body.coords.get(0).y);
  float M_y = P_x(body,p,i)*body.orth[i].l*(body.coords.get(i).x-body.coords.get(0).x);
  return M_x+M_y;
}
float M_Sum(Body body, Field p){
  float Msum = 0;
  for (int i=0; i<body.n; i++){
    Msum += M(body, p, i);
    //println("Px= "+P_x(body, p, i)+" Py= "+body.orth[i].l);
  }
  return Msum;
}
float P_input(float M_Sum,int time,float Max_theta, float Rotate_omega){
  return -M_Sum*Max_theta*Rotate_omega*cos(Rotate_omega*time);
}
float eff(float Psum, float Msum){
  return Psum/Msum;
}
