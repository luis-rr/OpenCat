#include <IRremote.h>

// abbreviation //gait/posture/function names
#define K00 "d" // rest and shutdown all servos
#define K01 "F" // forward
#define K02 "g" // turn off gyro feedback to boost speed

#define K10 "L"       // left
#define K11 "balance" // neutral stand up posture
#define K12 "R"       // right

#define K20 "p" // pause motion and shut off all servos
#define K21 "B" // backward
#define K22 "c" // calibration mode with IMU turned off

#define K30 "vt" // stepping
#define K31 "cr" // crawl
#define K32 "wk" // walk

#define K40 "tr" // trot
#ifdef NYBBLE
#define K41 "lu"     // look up
#define K42 "buttUp" // butt up
#else                // BITTLE
#define K41 "rn"     // run
#define K42 "ck"     // check around
#endif

#define K50 "hi"  // greeting
#define K51 "pu"  // push up
#define K52 "pee" // standng with three legs

#define K60 "str"  // stretch
#define K61 "sit"  // sit
#define K62 "zero" // zero position

#define CMD_LEN 10

class IRHandler {
public:
  decode_results results;
  char token;
  char lastToken;
  char *lastCmd = new char[CMD_LEN];
  char *newCmd = new char[CMD_LEN];
  byte newCmdIdx = 0;

  void setLastCmd(const char *const contents) { strcpy(lastCmd, contents); }

  bool isLastCmd(const char *const contents) const {
    return strcmp(lastCmd, "c");
  }

  void setNewCmd(const char *const contents) { strcpy(newCmd, contents); }

  bool isNewCmd(const char *const contents) const {
    return strcmp(newCmd, "c");
  }

  String translateIR() {
    // takes action based on IR code received
    // describing Remote IR codes.

    switch (results.value) {
    // IR signal    key on IR remote           //key mapping
    case 0xFFA25D: /*PTLF(" CH-");   */
      return (F(K00));
    case 0xFF629D: /*PTLF(" CH");  */
      return (F(K01));
    case 0xFFE21D: /*PTLF(" CH+"); */
      return (F(K02));

    case 0xFF22DD: /*PTLF(" |<<"); */
      return (F(K10));
    case 0xFF02FD: /*PTLF(" >>|"); */
      return (F(K11));
    case 0xFFC23D: /*PTLF(" >||"); */
      return (F(K12));

    case 0xFFE01F: /*PTLF(" -");   */
      return (F(K20));
    case 0xFFA857: /*PTLF(" +");  */
      return (F(K21));
    case 0xFF906F: /*PTLF(" EQ"); */
      return (F(K22));

    case 0xFF6897: /*PTLF(" 0");  */
      return (F(K30));
    case 0xFF9867: /*PTLF(" 100+"); */
      return (F(K31));
    case 0xFFB04F: /*PTLF(" 200+"); */
      return (F(K32));

    case 0xFF30CF: /*PTLF(" 1");  */
      return (F(K40));
    case 0xFF18E7: /*PTLF(" 2");  */
      return (F(K41));
    case 0xFF7A85: /*PTLF(" 3");  */
      return (F(K42));

    case 0xFF10EF: /*PTLF(" 4");  */
      return (F(K50));
    case 0xFF38C7: /*PTLF(" 5");  */
      return (F(K51));
    case 0xFF5AA5: /*PTLF(" 6");  */
      return (F(K52));

    case 0xFF42BD: /*PTLF(" 7");  */
      return (F(K60));
    case 0xFF4AB5: /*PTLF(" 8");  */
      return (F(K61));
    case 0xFF52AD: /*PTLF(" 9");  */
      return (F(K62));

    case 0xFFFFFFFF:
      return (""); // Serial.println(" REPEAT");

    default: {
      // Serial.println(results.value, HEX);
    }
    }
    return (""); // Serial.println("null");
    // delay(100); // Do not get immediate repeat //no need because the main
    // loop is slow

    // The control could be organized in another way, such as:
    // forward/backward to change the gaits corresponding to different speeds.
    // left/right key for turning left and right
    // number keys for different postures or behaviors
  }
};
