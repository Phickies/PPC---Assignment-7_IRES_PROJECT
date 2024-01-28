/*
  Parent UI class
*/

class User_Interface {

  private float posX, posY, UIwidth, UIheight,smoothCorner;
  private String content;
  private boolean ellipse;
  private int textSize = 20;                                          // Default text size
  private color fillColor;                                            
  private color textColor;                                           
  private PImage image;
  
  private PFont font = createFont("Roboto-Regular.ttf", this.textSize, true); // You could use any other font, just need to  download the font and paste that into 
  // the same folder having your main file, or put it into the data folder.
  
  User_Interface(){}
  protected User_Interface(boolean  ellipse, 
                           float    posX, 
                           float    posY, 
                           float    UIwidth, 
                           float    UIheight,
                           float    smoothCorner,
                           color    fillColor,
                           String   content, 
                           int      textSize,
                           color    textColor,
                           PImage   image) 
  {
    this.ellipse = ellipse;
    this.posX = posX;
    this.posY = posY;
    this.UIwidth = UIwidth;
    this.UIheight = UIheight;
    this.smoothCorner = smoothCorner;
    this.content = content;
    this.textSize = textSize;
    this.image = image;
    this.fillColor = fillColor;
    this.textColor = textColor;
  }
  
  protected void setPosX(float posX){
    this.posX = posX;
  }
  
  protected void setPosY(float posY){
    this.posY = posY;
  }
  
  protected void setFillColor(int red, int green, int blue) {
    this.fillColor = color(red, green, blue);
  }

  protected void setFillColor(color fillColor) {
    this.fillColor = fillColor;
  }

  protected void setContentColor(color textColor) {
    this.textColor = textColor;
  }

  protected void setTextSize(int textSize) {
    if (content != null) {
      this.textSize = textSize;
    }
  }
  
  protected void setText(String content){
    this.content = content;
  }
  
  protected float getPosX(){
    return posX;
  }
  
  protected float getPosY(){
    return posY;
  }
  
  protected color getFillColor(){
    return fillColor;
  }
  
  protected color getTextColor(){
    return textColor;
  }
  
  protected String getText(){
    return content;
  }
  
  protected boolean isEllipse(){
    return ellipse;
  }

  public void display() {
    if (isEllipse() != true){
      fill(getFillColor());
      if (image == null) {
        rect(posX, posY, UIwidth, UIheight, smoothCorner);
      } else {
        image(image, posX, posY, UIwidth, UIheight);
      }
      fill(getFillColor());
    
      if (content != null) {
        fill(textColor);
        textFont(font);
        textSize(textSize);
        text (content, posX, posY); 
        fill(textColor);
      }
    } else {
      fill(getFillColor());
      ellipse(posX, posY, UIwidth, UIheight);  
      fill(getFillColor());
    
      if (content != null) {
        fill(textColor);
        textFont(font);
        textSize(textSize);
        text (content, posX, posY); 
        fill(textColor);
      }
    }
  }
  
  public boolean isHover() {
    boolean isHover = false;
    if (isEllipse() != true){
      if ( mouseX > posX - UIwidth && mouseX < posX + UIwidth && mouseY > posY - UIheight && mouseY < posY + UIheight) {
        isHover = true;
      } else {
        isHover = false;
      }
    } else {
      if ((mouseX - posX)*(mouseX - posX) + (mouseY - posY)*(mouseY - posY) < (UIwidth*UIheight)){
        isHover = true;
      } else {
        isHover = false;
      }
    }
    return isHover;
  }
}
