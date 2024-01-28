/*
  Slider class
*/

public class Slider extends User_Interface{
  
  private float barWidth, barHeight, barPosX, barPosY;
  private float minValue, maxValue;
  private color barColor;
  private boolean horizontal;
  
  JSONObject json;
  
  /*
    void display();
    void matchValue(float value, float minValue, float maxValue);
    float updateValue(float mouseXY);
  */
  
  public Slider(boolean  ellipse,                // shape of the slider knob
                float    barPosX,                // position of slider bar on X axis
                float    barPosY,                // position of slider bar on Y axis
                float    barWidth,               // width of the slider bar
                float    barHeight,              // height of the slider bar
                color    barColor,               // color of the slider bar
                float    Slwidth,                // width of the knob
                float    Slheight,               // height of the knob
                float    Slsmooth,               // smoother the knob
                color    fillColor,              // color of the knob
                boolean  horizontal)             // horizontal or vertical
  {   
    super(ellipse, barPosX, barPosY, Slwidth, Slheight, Slsmooth, fillColor, null, 0, 0, null);
    this.barPosX = barPosX;
    this.barPosY = barPosY;
    this.barWidth = barWidth;
    this.barHeight = barHeight;
    this.barColor = barColor;
    this.horizontal = horizontal;
  }
  
  public void setBarColor(int red, int green, int blue){
    this.barColor = color(red, green, blue);
  }
  
  public void setBarColor(color newColor){
    this.barColor = newColor;
  }
  
  public color getBarColor(){
    return barColor;
  }
  
  public void display(){
    
    fill(barColor);
    if (horizontal){
      ellipse(barPosX - barWidth, barPosY, barHeight, barHeight);
      ellipse(barPosX + barWidth, barPosY, barHeight, barHeight);
    } else {
      ellipse(barPosX, barPosY - barHeight, barWidth, barWidth);
      ellipse(barPosX, barPosY + barHeight, barWidth, barWidth);
    }
    rect(barPosX, barPosY, barWidth, barHeight);
    fill(barColor);
    super.display();
  }
  
  public void matchValue(float value, float minValue, float maxValue){
    
    /*
      This function will map the slider with the value you want to adjust
      You will have to pass the min value and the max value.
    */
    
    this.minValue = minValue;
    this.maxValue = maxValue;
    if (horizontal){
      super.setPosX(map(value, minValue, maxValue, barPosX - barWidth, barPosX + barWidth));
    } else {
      super.setPosY(map(value, minValue, maxValue, barPosY - barHeight, barPosY + barHeight));
    } 
  }
  
  public float updateValue(float mouseXY){
    
    /*
      This function will update the value when you slide the silder.
      The parameter is the input of the changing value. 
      For example: 
        - the position of the mouse.
    */
    
    float newValue;
    if (horizontal){
      super.setPosX(mouseXY);  
      if (mouseXY < barPosX - barWidth){
        super.setPosX(barPosX - barWidth);
      } else if (mouseXY > barPosX + barWidth){
        super.setPosX(barPosX + barWidth);
      }
      newValue = map(super.getPosX(), barPosX - barWidth, barPosX + barWidth, this.minValue, this.maxValue);
    } else {
      super.setPosY(mouseXY);  
      if (mouseXY < barPosY - barHeight){
        super.setPosY(barPosY - barHeight);
      } else if (mouseXY > barPosY + barHeight){
        super.setPosY(barPosY + barHeight);
      }
      newValue = map(super.getPosY(), barPosY - barHeight, barPosY + barHeight, this.minValue, this.maxValue);
    }
    return newValue;
  }
  
  public void saveState(){
    json = new JSONObject();
    
    if (horizontal){
      json.setFloat("valueX", getPosX());
      saveJSONObject(json, "data/sliderXData.json");
    } else {
      json.setFloat("valueY",getPosY());
      saveJSONObject(json, "data/sliderYData.json");
    }   
  }
  
  public void loadState(){
    try{
      if (horizontal){
        json = loadJSONObject("data/sliderXData.json");
        setPosX(json.getFloat("valueX"));
        updateValue(json.getFloat("valueX"));
      } else {
        json = loadJSONObject("data/sliderYData.json");
        setPosY(json.getFloat("valueY"));  
        updateValue(json.getFloat("valueY"));
      }
    } catch (Exception error){
      error.printStackTrace();
    }
  }
}
