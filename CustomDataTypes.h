

class Vector3
{  
  public:
    float x;
    float y;
    float z;
};

Vector3 StringToVector3( String PassedString )                                                //-- = 10,12,7
{
  String X = PassedString.substring(0, PassedString.indexOf(","));                            //-- = 10
  String Temp = PassedString.substring(PassedString.indexOf(",")+1, PassedString.length());   //-- = 12,7
  String Y = Temp.substring(0, PassedString.indexOf(","));                                    //-- = 12
  String Z = PassedString.substring(PassedString.indexOf(",")+1, PassedString.length());      //-- = 7

  Vector3 OutputValue;
  OutputValue.x = X.toInt();
  OutputValue.y = Y.toInt();
  OutputValue.z = Z.toInt();

  return(OutputValue);
}


class VectorWaypoint
{  
  public:
    Vector3 Position;
    Vector3 Rotation;
};


class InternalBattery_CLASS
{
  private:
    float Empty_Voltage = 3.40;
    float Full_Voltage = 3.70;
    float Current_Voltage;
    int   Charge_Percentage;
    int   Charge_Quarters;
  
  public:
    float READ_Current_Voltage(){
      Current_Voltage = (random(340, 370))/100.00;
      return(Current_Voltage);
    }
    
    int GET_Charge_Percentage(){
      // Map int   y = map(x, a, b, c, d);
      // Map float y = (x/(b-a)*(d-c)+c);
      int x = (100*READ_Current_Voltage());
      int a = (100*Empty_Voltage);
      int b = (100*Full_Voltage);
      int c = 0;
      int d = 100;
      Charge_Percentage = map(x, a, b, c, d);
      return(Charge_Percentage);
    }

    int GET_Charge_Quarters(){
      int x = GET_Charge_Percentage();
      int a = 0;
      int b = 100;
      int c = 0;
      int d = 4;
      Charge_Quarters = map(x, a, b, c, d);
      return(Charge_Quarters);
    }
};
