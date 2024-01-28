public class SceneManager {

  private color backgroundColor;
  private boolean isConnected;
  private boolean errorNotConnected;
  private String[] robotData; // To store parsed data
  private boolean firstContact;

  Button connectButton;

  public SceneManager(color backgroundColor) {
    this.backgroundColor = backgroundColor;
    this.isConnected = false;
    this.errorNotConnected = false;
    this.firstContact = false;

    connectButton = new Button(false, width/2, height/2+50, 70, 20, 20,
      color(210), "CONNECT", 20, color(0), null);
  };

  /**
   * Display the waiting scene for connection
   */
  public void waitingForConnection() {
    if (!isConnected) {
      background(backgroundColor);
      fill(255);
      textSize(80);
      text("SERI", width/2, 200);
      textSize(30);
      text("Support Emotional Robot Intelligent", width/2, 300);
      connectButton.display();
      connectButton.hoverAnimation(200, 200, 0);
    }
  }

  /**
   * Method to handle data updates
   *
   * Establish handshake protocol to the Arduino to wait for connection.
   *
   * If connection, parse the data into RobotData variable.
   */
  public void updateWithData(Serial myPort) {
    // Update this part to continuously check for new data and display it
    try {
      if (myPort.available() > 0) {
        String data = myPort.readStringUntil('\n'); // Read data from serial
        if (data != null) {
          String val = trim(data); // Trim whitespace and newline
          robotData = split(val, ','); // Split the comma-separated values

          // look for our 'A' string to start the handshake
          // if it's there, clear the buffer,
          // and send a request for data
          if (!firstContact) {
            if (val.equals("A")) {
              myPort.clear();
              firstContact = true;
              myPort.write("B");
              println("contact");
            }
          } else {
            // if we've already established contact
            // keep getting and parsing data
            for (int i = 0; i < robotData.length; i++) {
              println(robotData[i]);
            }

            // when you've parsed the data you have, ask for more
            myPort.write("B");
          }
        }
      }
    }
    catch(Exception e) {
      println("Serial port initialization error: "
        + e.getMessage());
    }
  }

  /**
   * Display the scene based on Robot Data
   */
  public void showMainScene() {

    if (isConnected) {
      if (robotData != null && robotData.length >= 2) {
        // Parse your data
        String emotionalState = robotData[0];
        int brightness = int(robotData[1]);

        // Update display based on the emotional state and brightness
        background(brightness);  // Adjust background based on brightness
        fill(255);               // Set text color
        textSize(20);
        text("Emotional State: " + emotionalState, width/2, height/2);
        // Additional display updates here
      }
    }
  }

  /**
   * Event for changing Scence
   */
  public void connectedButtonPressedEvent() {
    if (connectButton.isHover()) {
      if (errorNotConnected) {
        fill(255, 0, 0);
        text("There is no SERI hooked up", width/2, 700);
      } else {
        isConnected = true;
        println("CONNECTED WITH SERI SUCCESSFULLY");
      }
    }
  }
}
