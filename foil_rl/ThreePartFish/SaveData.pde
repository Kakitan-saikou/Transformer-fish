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
     
  void saveParam(String name, float value){
    output.println("% " + name + " = " + value + ";");
  }
  void saveString(String s){
    output.println(s);
  }
  
  void addData(float t, Field a){
    output.print(t + " ");
    for(int i=0; i<n; i++){
      output.print(a.linear( coords.get(i).x, coords.get(i).y ) +" ");
    }
    output.println(";");
  }
  
  void addData(float t, PVector p, Body b, Field a){
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
  
  void addDataSimple(float t, PVector p, Body b, Field a){  //simplified to only output time and vector x and y values 
    output.print(t + " ");
    output.print(p.x + " " + p.y + " ");
    output.println(";");
  }

  void addText(String s){   //allows to insert text anywhere in txt
    output.println(s);
  }

  void finish(){
    output.flush(); // Writes the remaining data to the file
    output.close(); // Finishes the file
  }
  void addFxSum(float t, Body b, Field a,float Max_theta, float Rotate_omega){ 
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
  
 
