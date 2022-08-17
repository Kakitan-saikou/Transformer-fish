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
  float k, omega, T, x0, alpha, lamda, time=0, a0, xxx=10.0,fL,wf;
  ArrayList<PVector> tcoords=new ArrayList<PVector>();
  float [] xm=new float [2*m];
  float dt = 0.5;
  
  NACA orig;
     

  Flexfish( float x, float y, float _fL, float _T, float a1, float _wf, Window window ) {
  
    // set the NACA coords and save as orig
    super( x, y, _fL, _wf, 0,window );
    orig = new NACA( x, y, _fL,_wf,0, window );

    // set the wave parameters
    a0 = a1;
    alpha = -0.5;
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
  float distance( float x, float y ) { // shift y and use orig distance
    float dis = 1e10;
      for( OrthoNormal o: orth ) dis = min(dis,o.distance(x,y,false));
      return (wn(x,y)==0)?dis:-dis; 
    //return orig.distance( x-si((x-xc.x)/fL)*wf*fL*offset((x-xc.x)/fL) , y);
    
  }
  
  PVector WallNormal(float x, float y  ) { // shift y and adjust orig normal
    PVector wnormal = new PVector(0, 0);
    float dis = -1e10;
    float dis2 = -1e10;
 
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
  
    
  
  float velocity( int d, float dt, float x, float y ) { // use wave velocity
  
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

  void update(float time) { // update time and coords
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
  
  boolean unsteady() {
    return true;
  }




  float w( float x){
    return Y(x)*sin(TWO_PI*x/lamda-time/T*TWO_PI);
    
  }

  float Y( float x){
    return ae(x)*exp(-alpha)*abs((exp(alpha*x)-1));
  }
  float ae(float x){
    return a0*exp(alpha)/abs(exp(alpha)-1);
  }
  float dwdx( float x){
    return a0/abs(exp(alpha)-1)*(alpha*exp(alpha*x)*sin(TWO_PI/lamda*x-time/T*TWO_PI)+(exp(alpha*x)-1)*cos(TWO_PI/lamda*x-time/T*TWO_PI)*TWO_PI/lamda);
  }
  float si( float x){
    return -sin(atan(dwdx(x)));
  }
  float co( float x){
    return  -cos(atan(dwdx(x)));
  }
  float dwdt( float x){
    return -Y(x)*cos(TWO_PI*x/lamda-time/T*TWO_PI)*TWO_PI/T;
  }
  float offset( float x ){
    return 5*(0.2969*sqrt(x)-0.1260*x-0.3516*pow(x,2)+0.2843*pow(x,3)-0.1015*pow(x,4));
  }
  float yloc(int i, float x){
  return (coords.get(i).y-coords.get(i-1).y)*(x-coords.get(i-1).x)/(coords.get(i).x-coords.get(i-1).x)+coords.get(i-1).y;
  }
  float tyloc(int i, float x){
  return (tcoords.get(i).y-tcoords.get(i-1).y)*(x-tcoords.get(i-1).x)/(tcoords.get(i).x-tcoords.get(i-1).x)+tcoords.get(i-1).y;
  }
  float vx(int i, float x){
  return (x-(x-coords.get(i-1).x)/(coords.get(i).x-coords.get(i-1).x)*(tcoords.get(i).x-tcoords.get(i-1).x)+tcoords.get(i-1).x)/dt;
  }
}
  
