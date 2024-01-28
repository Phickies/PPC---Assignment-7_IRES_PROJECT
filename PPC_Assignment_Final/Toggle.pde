/*
  Toggle class
*/

final public class Toggle extends Slider{
  
  private color offColor, onColor;
  private float barPosX, barWidth, switchX;
  private boolean switchState = true;
  private boolean end = false;
  private int animationTime = 5;
  
  JSONObject json;   
  
  public Toggle(float    barPosX,                // position of toggle bar on X axis
                float    barPosY,                // position of toggle bar on Y axis
                float    barWidth,               // width of the toggle bar
                color    barColor,               // color of the toggle bar when on
                color    offColor,               // color of the toggle bar when off
                color    fillColor)              // color of the switch
  {
    super(true, barPosX, barPosY, barWidth, barWidth, barColor, barWidth, barWidth, 0, fillColor, true);
    this.barPosX = barPosX;
    this.barWidth = barWidth;
    this.offColor = offColor;
    this.onColor = barColor;
    this.switchX = barPosX; 
  }
  
  public void setSwitchX(float switchX){
    this.switchX = switchX;
  }
  
  public void setSwitchState(boolean state){
    this.switchState = state;
  }
  
  public void setEndState(boolean state){
    this.end = state;
  }
  
  public void setAnimationTime(int value){
    this.animationTime = value;
  }
  
  public boolean getSwitchState(){
    return switchState;
  }
  
  public boolean getEndState(){
    return end;
  }
  
  public int getAnimationTime(){
    return animationTime;
  }
  
  public void display(){  
    super.display();
    this.update();
  }
  
  public void update(){
    if (switchState){
      if (!end){
        switchX -= animationTime;
        if (switchX == barPosX - barWidth){
          super.setBarColor(offColor);
          end = true;
        }
        super.setPosX(switchX);
      }
    } else {
      if (end){
        switchX += animationTime;
        if (switchX == barPosX + barWidth){
          super.setBarColor(onColor);
          end = false;
        }
        super.setPosX(switchX);
      }
    }
  }
  
  public boolean switched(){
    boolean switched = false;
    if (super.isHover()){
      switched = true;
    }
    return switched;
  }
  
  public void saveState(){
    json = new JSONObject();
    json.setFloat("switchPos", getPosX());
    json.setInt("barColor", getBarColor());
    json.setBoolean("switchState", getSwitchState());  
    json.setBoolean("endState", getEndState());
    saveJSONObject(json, "data/toggleData.json");
  }
  
  public void loadState(){
    try{
      json = loadJSONObject("data/toggleData.json");
      setPosX(json.getFloat("switchPos"));
      setSwitchX(json.getFloat("switchPos"));
      setBarColor(json.getInt("barColor"));
      setSwitchState(json.getBoolean("switchState"));
      setEndState(json.getBoolean("endState"));    
    } catch (Exception error){
      error.printStackTrace();
    } 
  }
}
