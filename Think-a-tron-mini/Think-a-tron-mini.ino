/**
 * Implements the Think-a-Tron Mini console interface.
 */
#include <SoftwareSerial.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1351.h>

#define DEBUG 1

// Screen dimensions
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 128

#define RST_PIN  8
#define DC_PIN   7
#define CS_PIN   10
#define MOSI_PIN 11
#define SCLK_PIN 13

// Color definitions
#define BLACK    0x0000
#define BLUE     0x001F
#define RED      0xF800
#define GREEN    0x07E0
#define CYAN     0x07FF
#define MAGENTA  0xF81F
#define YELLOW   0xFFE0 
#define WHITE    0xFFFF

// Setup the screen object.
Adafruit_SSD1351 oled = Adafruit_SSD1351(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, CS_PIN, DC_PIN, RST_PIN);

// Using a software serial port to allow debugging. Don't need high speed anyway.
#define RX 2
#define TX 3
SoftwareSerial mySerial(RX, TX);

// Define an input buffer to hold the data coming from the GM67 Bar Code Reader Module.
#define IN_BUFFER_SIZE  10
char inputBuffer[IN_BUFFER_SIZE];
int inBufferWrite = 0;
int inBufferRead = 0;

// Player information.
int player1Score = 0;
char player1Guess = ' ';
int player2Score = 0;
char player2Guess = ' ';

// Define two modes of operation.
#define QUESTION_MARK '?'
#define QUESTION_MODE 0
#define ANSWER_MODE 1
int mode = QUESTION_MODE;

// Show animation?
boolean showAnimation = true;

// Send the starting sound on command to invoke the sound.
char startingSoundOn[] = {0x08, 0xC6, 0x04, 0x08, 0x00, 0xF2, 0x0D, 0x01, 0xFE, 0x26};
char resetCommand[] = {0x04, 0xFA, 0x04, 0x00, 0xFE, 0xFE};

// How much time to show the answer.
unsigned long showAnswerTime = 5000;
unsigned long showAnswerStartTime = 0;

// Information about commands read from the QR cards.
#define COMMAND_SIZE 3

// Get everything ready to run.
void setup() {
  // Setup the pins for software serial and start the interface.
  pinMode(2, INPUT);
  pinMode(3, OUTPUT);
  mySerial.begin(9600);

  // Init SPI
  SPI.setDataMode(SPI_MODE0);
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.begin();

  // Start the screen.
  oled.begin(); 
  oled.fillRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, BLACK);
  drawMainLetter(QUESTION_MARK, WHITE);
  drawScores(0, 0);

#ifdef DEBUG
  // Open serial communications and wait for port to open.
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Native USB only
  }
  Serial.println("Think-a-Tron Mini Ready!");
#endif
}

// Process input from the GM67 Bar Code Reader Module. Implements a Think-a-Tron trivia game.
void loop() {
  while (mySerial.available() > 0) {
    char c = mySerial.read();
#ifdef DEBUG
    Serial.write(c);
    Serial.println();
#endif
    // Add the character read to the input buffer.
    writeToBuffer(c); 
  }
  
  // See if there are enough characters in the input buffer to form a command.
  if (mode == QUESTION_MODE && charactersInBuffer() >= COMMAND_SIZE) {
    
    // Process the next command. 
    char c = readFromBuffer();

    // See what type of command it is.
    if (c == 'P') {
      // Player card. 
      char player = readFromBuffer();
      char guess = readFromBuffer();
      if (player == '1') {
        player1Guess = guess;
        drawPlayer1Guess(guess, WHITE);
      } else {
        player2Guess = guess;
        drawPlayer2Guess(guess, WHITE);
      }
    } else if (c == 'C') {
      // Control card.
      int code = (readFromBuffer() - '0') * 10 + readFromBuffer() - '0';
      switch (code) {
        case 0: 
          player1Score = 0;
          player2Score = 0;
          drawScores(0, 0);
          drawMainLetter(QUESTION_MARK, WHITE);
          drawPlayer1Guess(' ', WHITE);
          drawPlayer1Guess(' ', WHITE);
          break;
        case 1: 
          showAnimation = false;
          break;
        case 2:
          showAnimation = true;
          break;
        case 3:
          showAnswerTime = 5000;
          break;
        case 4:
          showAnswerTime = 10000;
          break;
        case 5:
          showAnswerTime = 15000;
          break;
        default: 
          break;
      }
    } else if (c == 'A') {
      // Answer card. Figure out what the answer was.
      int firstDigit = readFromBuffer() - '0';
      int secondDigit = readFromBuffer() - '0';
      int answer = (firstDigit + secondDigit) % 10;
      char answerLetter = getAnswerLetter(answer);

      // Show an animation.
      showRandom();
      
      // Show the answer.
      drawMainLetter(answerLetter, WHITE);

      // Check the scores.
      if (answerLetter == player1Guess) {
        player1Score += 1;
        drawPlayer1Guess(player1Guess, GREEN);
      } else {
        drawPlayer1Guess(player1Guess, RED);
      }
      if (answerLetter == player2Guess) {
        player2Score += 1;
        drawPlayer2Guess(player2Guess, GREEN);
      } else {
        drawPlayer2Guess(player2Guess, RED);
      }
      drawScores(player1Score, player2Score);

      // Set to answer mode.
      mode = ANSWER_MODE;
      
      // Remember when we started showing the answer.
      showAnswerStartTime = millis();

      // Wait for something to happen.
      delay(100);
    }
  } else if (mode == ANSWER_MODE) {
    unsigned long checkTime = millis();
    if ((checkTime - showAnswerStartTime) > showAnswerTime) {
      // Switch back to question mode.
      player1Guess = ' ';
      player2Guess = ' ';
      drawPlayer1Guess(player1Guess, WHITE);
      drawPlayer2Guess(player2Guess, WHITE);
      drawMainLetter(QUESTION_MARK, WHITE);
      mode = QUESTION_MODE;
    }
  }

  // Wait for something to happen.
  delay(100);
}

// Write a character to the input buffer.
void writeToBuffer(char c) {
  inputBuffer[inBufferWrite] = c;
  inBufferWrite = (inBufferWrite + 1) % IN_BUFFER_SIZE; 
}

// Read a character from the input buffer.
char readFromBuffer() {
  char c = inputBuffer[inBufferRead];
  inBufferRead = (inBufferRead + 1) % IN_BUFFER_SIZE;
  return c;
}

// Based on the answer number return the answer letter.
char getAnswerLetter(int answer) {
  char playerAnswer = ' ';
  switch (answer) {
    case 1: playerAnswer = 'A'; break;
    case 2: playerAnswer = 'B'; break;
    case 3: playerAnswer = 'C'; break;
    case 4: playerAnswer = 'D'; break;
    case 5: playerAnswer = 'E'; break;
    case 6: playerAnswer = 'T'; break;
    case 7: playerAnswer = 'F'; break;
  }
  return playerAnswer;
}

// Determine how many characters are currently in the input buffer.
int charactersInBuffer() {
  if (inBufferWrite > inBufferRead) {
    return inBufferWrite - inBufferRead;
  } else if (inBufferWrite < inBufferRead) {
    return (IN_BUFFER_SIZE - inBufferRead) + inBufferWrite;
  }
  return 0;
}

// Draw a large blocky letter in the center of the screen.
void drawMainLetter(char letter, uint16_t color) {
  oled.drawChar(37, 30, letter, color, BLACK, 11);
}

// Draw thw score for one of the players.
void drawScores(int score1, int score2) {
  
  oled.setTextColor(YELLOW, BLACK);
  oled.setTextSize(2);
  oled.setCursor(0, 10);
  if (score1 < 10) oled.print(0);
  oled.print(score1);
  oled.setCursor(104, 10);
  if (score2 < 10) oled.print(0);
  oled.print(score2);
}

// Draw the guess for player one.
void drawPlayer1Guess(char guess, uint16_t color) {
  oled.setTextColor(color, BLACK);
  oled.setTextSize(2);
  oled.setCursor(0, 114);
  oled.print(guess);
}

// Draw the guess for player two.
void drawPlayer2Guess(char guess, uint16_t color) {
  oled.setTextColor(color, BLACK);
  oled.setTextSize(2);
  oled.setCursor(116, 114);
  oled.print(guess);
}

// Show a random digital pattern in the letter area.
void showRandom() {
  if (showAnimation) {
    for (int repeats = 0; repeats < 100; repeats++) {
      for (int row = 0; row < 7; row++) {
        for (int col = 0; col < 5; col++) {
          int xOffset = 37 + col * 11;
          int yOffset = 30 + row * 11;
          uint16_t color = BLACK;
          if (random(3) == 0) color = WHITE; 
          oled.fillRect(xOffset, yOffset, 11, 11, color);
        }
      }
    }
  }
}

// Send a command to the GM67.
void sendCommand(char command[]) {
  // Wake up the device.
  char wakeUpCommand = 0x00;
  mySerial.write(wakeUpCommand);
  delay(50);
  for (int i = 0; i < sizeof(command); i++) {
    mySerial.write(command[i]);
  }
}
