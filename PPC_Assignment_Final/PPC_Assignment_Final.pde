import processing.serial.*;
import processing.net.*;

Serial myPort;
Server server;
Client client;
SceneManager sceneManager;

color backgroundColor = 0;

void setup() {
  size(800, 800);
  noStroke();
  rectMode(RADIUS);
  ellipseMode(RADIUS);
  textAlign(CENTER, CENTER);
  frameRate(200);

  server = new Server(this, 36969);
  sceneManager = new SceneManager(0);

  try {
    myPort = new Serial(this, "COM5", 9600);
  }
  catch (Exception e) {
    println("Serial port initialization error: "
      + e.getMessage());
    sceneManager.errorNotConnected = true;
  }
}

void draw() {
  background(0);
  sceneManager.waitingForConnection();
  sceneManager.updateWithData(myPort);
  sceneManager.showMainScene();
}

void mousePressed() {
  sceneManager.connectedButtonPressedEvent();
}
