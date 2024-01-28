/*
  Button class:
*/

final public class Button extends User_Interface{
  
  private color tempColor;
  private color tempTextColor;
  private boolean buttonState = true, animation = false;
  
  JSONObject json;
  
  /*
    public void hoverAnimation(int red, int green, int blue);
    public boolean isPressed();
  */
  
  // For the image parameter, you have to pass function //loadImage("Name of your picture file")
  public Button(boolean  ellipse,                 // shape of the button is ellipse or rectagle
                float    posX,                    // position of the button on X axis
                float    posY,                    // position of the button on Y axis
                float    Bwidth,                  // width of the button
                float    Bheight,                 // height of the button
                float    smoothCorner,            // smoother the corner of the button
                color    fillColor,               // color of the button
                String   content,                 // text on the button (if not pass null)
                int      textSize,                // size of the text   (if not pass 0)
                color    textColor,               // color of the text  (if not pass 0)
                PImage   image)                   // image on the button(if not pass null)
   {
     super(ellipse, posX, posY, Bwidth, Bheight, smoothCorner, fillColor, content, textSize, textColor, image);
     tempColor = super.getFillColor();
     tempTextColor = super.getTextColor();
   }
   
   public void setColor(color newColor){
     if (animation){
       this.tempColor = newColor;
     } else {
       super.setFillColor(newColor);
     }
   }
   
   public void setColor(int red, int green, int blue){
     if (animation){
       this.tempColor = color(red, green, blue);
     } else {
       super.setFillColor(color(red, green, blue));
     }
   }
   
   public void setTextColor(color newColor){
     if (animation){
       this.tempTextColor = newColor;
     } else {
       super.setContentColor(newColor);
     }
   }
   
   public void setButtonState(boolean state){
     buttonState = state;
   }
   
   public boolean getButtonState(){
     return buttonState;
   }
   
   public void display(){
     super.display();
   }
   
   public void hoverAnimation(int red, int green, int blue){
     
    /* 
      This function will animated the button when you hover your mouse over it
      The parameter pass the value of the color you want to changen to
    */
    
     if (super.isHover()){
       super.setFillColor(color(red, green, blue));
       super.setContentColor(0);
     } else {
       super.setFillColor(this.tempColor);
       super.setContentColor(this.tempTextColor);
     }
     animation = true;
   }
   
   public boolean isPressed(){
     boolean isPressed = false;
     if (super.isHover()){
       isPressed = true;
     }
     return isPressed;
   }
   
   public void saveState(){
    json = new JSONObject();   
    json.setInt("fillColor", getFillColor());
    json.setInt("textColor", getTextColor());
    json.setString("content", getText());
    json.setBoolean("buttonState", getButtonState());  
    saveJSONObject(json, "data/buttonData.json");
   }
   
   public void loadState(){
     try{
      json = loadJSONObject("data/buttonData.json");
      setFillColor(json.getInt("fillColor"));
      setTextColor(json.getInt("textColor"));
      setText(json.getString("content"));
      setButtonState(json.getBoolean("buttonState"));  
    } catch (Exception error){
      error.printStackTrace();
    }
   }
   
   public void mousePressedEvent(){
   }
}
