#ifndef   XML_PARSE_H_
#define   XML_PARSE_H_

#include <libxml/parser.h>
#include <libxml/tree.h>

#include <cstdint>

struct XmlMatrix4x4
{
  double row1[4];
  double row2[4];
  double row3[4];
  double row4[4];
};

struct XmlPI
{
  float kp;
  float ki;
};

struct XmlPID
{
  double kp;
  double ki;
  double kd;
};

struct XmlBDCMotor
{
  int32_t id;
  int32_t gid;
  int32_t board;
  int32_t port;   // 0: L6205_A; 1: L6205_B
  XmlPI   xmlPI;
  XmlPID  xmlPID;
  int32_t resolution;   // encoder counters per 2PI
  double reduction;
  double maxCurrent;    // amp
};

struct XmlInstrument
{
  char name[128];
  int32_t ecindex;    // ethercat master index
  int32_t type;
  XmlBDCMotor motor[3];
};

struct XmlKuka
{
  char name[128];
  char stationAddress[128];
  int32_t stationPort;    // src port of kuka
  char koniAddress[128];
  int32_t friPort;
  int      controlMode ;    // 0: approach motion, 1: joint control
  double   a6BestDeg;
};

struct XmlLeftPSM
{
  char name[128];
  XmlMatrix4x4 baseInCart;
};

struct XmlRightPSM
{
  char name[128];
  XmlKuka kuka;
  XmlInstrument instrument;
  XmlMatrix4x4 baseInCart;
};

struct XmlCart
{
  XmlLeftPSM   left;
  XmlRightPSM  right;
  int32_t kukaListenPort;
};

struct XmlAmpsToBits
{
  int32_t  offset;
  double   scale;
};

struct XmlDrive
{
  XmlAmpsToBits  ampsToBits;
  double maxAmps;
};

struct XmlEncoder
{
  double BitsToPosSI;
};

struct XmlAnalog
{
  int32_t    min;
  int32_t    max;
  int32_t    zero;
  int32_t    scale;
};

struct XmlDvrkAxis
{
  int32_t    id;
  int32_t    board;
  int32_t    port;
  int8_t     ampmask;
  XmlDrive   drive;
  XmlEncoder encoder;
  XmlAnalog  analog;
};

struct XmlMTM
{
  char           name[128];
  XmlDvrkAxis    axis[8];
  XmlMatrix4x4   baseInConsole;
};

struct XmlConsole
{
  XmlMTM  mtms[2];  // left and right
};

class XmlParser
{
public:
  XmlParser(const char* configFile);
  virtual ~XmlParser();

public:
  const XmlConsole& GetConsole() const { return mConsole; };
  const XmlCart& GetCart() const { return mCart; };

private:
  void ParseConsole(xmlDocPtr pDoc, xmlNodePtr pNode, XmlConsole &console);
  void ParseCart(xmlDocPtr pDoc, xmlNodePtr pNode, XmlCart &cart);
  void ParseLeftPSM(xmlDocPtr pDoc, xmlNodePtr pNode, XmlLeftPSM &psm);
  void ParseRightPSM(xmlDocPtr pDoc, xmlNodePtr pNode, XmlRightPSM &psm);
  void ParseKuka(xmlDocPtr pDoc, xmlNodePtr pNode, XmlKuka &kuka);
  void ParseInstrument(xmlDocPtr pDoc, xmlNodePtr pNode, XmlInstrument &instrument);
  void ParseBDCMotor(xmlDocPtr pDoc, xmlNodePtr pNode, XmlBDCMotor &motor);
  void ParsePI(xmlDocPtr pDoc, xmlNodePtr pNode, XmlPI &pi);
  void ParsePID(xmlDocPtr pDoc, xmlNodePtr pNode, XmlPID &pid);
  void ParseMTM(xmlDocPtr pDoc, xmlNodePtr pNode, XmlMTM &mtm);
  void ParseAxis(xmlDocPtr pDoc, xmlNodePtr pNode, XmlDvrkAxis &axis);
  void ParseMatrix4x4(xmlDocPtr pDoc, xmlNodePtr pNode, XmlMatrix4x4 &matrix);
  void ParseDrive(xmlDocPtr pDoc, xmlNodePtr pNode, XmlDrive &drive);
  void ParseEncoder(xmlDocPtr pDoc, xmlNodePtr pNode, XmlEncoder &encoder);
  void ParseAmpsToBits(xmlDocPtr pDoc, xmlNodePtr pNode, XmlAmpsToBits &ampsToBits);
  void ParseAnalog(xmlDocPtr pDoc, xmlNodePtr pNode, XmlAnalog &analog);
  int ParseInt(xmlDocPtr pDoc, xmlNodePtr pNode, int* pValue);
  int ParseDouble(xmlDocPtr pDoc, xmlNodePtr pNode, double* pValue);
  int ParseFloat(xmlDocPtr pDoc, xmlNodePtr pNode, float* pValue);

  XmlConsole    mConsole;
  XmlCart       mCart;
};

#endif

