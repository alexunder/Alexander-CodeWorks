#include "XmlParser.h"

#include <string.h>
#include <error.h>

XmlParser::XmlParser(const char* configFile)
{
  xmlDocPtr pDoc = xmlReadFile(configFile, "UTF-8", 256);
  if (NULL == pDoc) {
    return;
  }

  xmlNodePtr pNode = xmlDocGetRootElement(pDoc);
  if (NULL == pNode) {
    xmlFreeDoc(pDoc);
    return;
  }

  if (0 != xmlStrcmp(pNode->name, (const xmlChar*)"KayakController")) {
    xmlFreeDoc(pDoc);
    return;
  }

  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Console")) {
      ParseConsole(pDoc, pNode, mConsole);
    }
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Cart")) {
      ParseCart(pDoc, pNode, mCart);
    }
    pNode = pNode->next;
  }
  xmlFreeDoc(pDoc);
  return;
}

XmlParser::~XmlParser()
{
}

void XmlParser::ParseConsole(xmlDocPtr pDoc, xmlNodePtr pNode, XmlConsole &console)
{
  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"MTM")) {
      xmlChar *pMTMId = xmlGetProp(pNode, (const xmlChar*)"id");
      if (NULL == pMTMId) {
        return;
      }
      int32_t mtmIdx = ((char*)pMTMId)[0] - '0';
      ParseMTM(pDoc, pNode, console.mtms[mtmIdx]);
      xmlFree(pMTMId);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseCart(xmlDocPtr pDoc, xmlNodePtr pNode, XmlCart &cart)
{
  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"PSMLeft")) {
      ParseLeftPSM(pDoc, pNode, cart.left);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"PSMRight")) {
      ParseRightPSM(pDoc, pNode, cart.right);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"KukaListenPort")) {
      ParseInt(pDoc, pNode, &cart.kukaListenPort);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseLeftPSM(xmlDocPtr pDoc, xmlNodePtr pNode, XmlLeftPSM &psm)
{
  strcpy(psm.name, (const char*)xmlGetProp(pNode, (const xmlChar*)"name"));

  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"BaseInCart")) {
      ParseMatrix4x4(pDoc, pNode, psm.baseInCart);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseRightPSM(xmlDocPtr pDoc, xmlNodePtr pNode, XmlRightPSM &psm)
{
  strcpy(psm.name, (const char*)xmlGetProp(pNode, (const xmlChar*)"name"));

  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Kuka")) {
      ParseKuka(pDoc, pNode, psm.kuka);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Instrument")) {
      ParseInstrument(pDoc, pNode, psm.instrument);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"BaseInCart")) {
      ParseMatrix4x4(pDoc, pNode, psm.baseInCart);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseKuka(xmlDocPtr pDoc, xmlNodePtr pNode, XmlKuka &kuka)
{
  strcpy(kuka.name, (const char*)xmlGetProp(pNode, (const xmlChar*)"name"));

  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"StationAddress")) {
      xmlChar* pValueStr = xmlNodeGetContent(pNode);
      if (NULL != pValueStr) {
        strcpy(kuka.stationAddress, (const char *)pValueStr);
        xmlFree(pValueStr);
      }
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"StationPort")) {
      ParseInt(pDoc, pNode, &kuka.stationPort);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"KONIAddress")) {
      xmlChar* pValueStr = xmlNodeGetContent(pNode);
      if (NULL != pValueStr) {
        strcpy(kuka.koniAddress, (const char *)pValueStr);
        xmlFree(pValueStr);
      }
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"FRIPort")) {
      ParseInt(pDoc, pNode, &kuka.friPort);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"ControlMode")) {
      ParseInt(pDoc, pNode, &kuka.controlMode);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"A6BestDeg")) {
      ParseDouble(pDoc, pNode, &kuka.a6BestDeg);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseInstrument(xmlDocPtr pDoc, xmlNodePtr pNode, XmlInstrument &instrument)
{
  strcpy(instrument.name, (const char*)xmlGetProp(pNode, (const xmlChar*)"name"));

  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"EcIndex")) {
      ParseInt(pDoc, pNode, &instrument.ecindex);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Type")) {
      ParseInt(pDoc, pNode, &instrument.type);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Motor")) {
      xmlChar *pMotorId = xmlGetProp(pNode, (const xmlChar*)"id");
      if (NULL == pMotorId) {
        // TODO: output error
        return;
      }
      int32_t motorIdx = ((char*)pMotorId)[0] - '0';
      ParseBDCMotor(pDoc, pNode, instrument.motor[motorIdx]);
      xmlFree(pMotorId);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseBDCMotor(xmlDocPtr pDoc, xmlNodePtr pNode, XmlBDCMotor &motor)
{
  xmlChar *pId = xmlGetProp(pNode, (const xmlChar*)"id");
  xmlChar *pGid = xmlGetProp(pNode, (const xmlChar*)"gid");
  xmlChar *pBoardId = xmlGetProp(pNode, (const xmlChar*)"board");
  xmlChar *pPortId = xmlGetProp(pNode, (const xmlChar*)"port");
  if (NULL == pId || NULL == pGid || NULL == pBoardId || NULL == pPortId) {
    // TODO: output error
    return;
  }
  int32_t id = ((char*)pId)[0] - '0';
  int32_t gid = ((char*)pGid)[0] - '0';
  int32_t boardId = ((char*)pBoardId)[0] - '0';
  int32_t portId = ((char*)pPortId)[0] - '0';
  motor.id = id;
  motor.gid = gid;
  motor.board = boardId;
  motor.port = portId;

  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"PI")) {
      ParsePI(pDoc, pNode, motor.xmlPI);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"PID")) {
      ParsePID(pDoc, pNode, motor.xmlPID);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Resolution")) {
      ParseInt(pDoc, pNode, &motor.resolution);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Reduction")) {
      ParseDouble(pDoc, pNode, &motor.reduction);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"MaxCurrent")) {
      ParseDouble(pDoc, pNode, &motor.maxCurrent);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParsePI(xmlDocPtr pDoc, xmlNodePtr pNode, XmlPI &pi)
{
  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Kp")) {
      ParseFloat(pDoc, pNode, &pi.kp);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Ki")) {
      ParseFloat(pDoc, pNode, &pi.ki);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParsePID(xmlDocPtr pDoc, xmlNodePtr pNode, XmlPID &pid)
{
  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Kp")) {
      ParseDouble(pDoc, pNode, &pid.kp);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Ki")) {
      ParseDouble(pDoc, pNode, &pid.ki);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Kd")) {
      ParseDouble(pDoc, pNode, &pid.kd);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseMTM(xmlDocPtr pDoc, xmlNodePtr pNode, XmlMTM &mtm)
{
  strcpy(mtm.name, (const char*)xmlGetProp(pNode, (const xmlChar*)"name"));

  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Axis")) {
      xmlChar *pAxisId = xmlGetProp(pNode, (const xmlChar*)"id");
      if (NULL == pAxisId) {
        // TODO: output error
        return;
      }
      int32_t axisIdx = ((char*)pAxisId)[0] - '0';
      ParseAxis(pDoc, pNode, mtm.axis[axisIdx]);
      xmlFree(pAxisId);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"BaseInConsole")) {
      ParseMatrix4x4(pDoc, pNode, mtm.baseInConsole);
    }
    pNode = pNode->next;
  }
  return;
}

void XmlParser::ParseMatrix4x4(xmlDocPtr pDoc, xmlNodePtr pNode, XmlMatrix4x4 &matrix)
{
  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Row")) {
      xmlChar *pRowId = xmlGetProp(pNode, (const xmlChar*)"id");
      xmlChar *pCol1 = xmlGetProp(pNode, (const xmlChar*)"col1");
      xmlChar *pCol2 = xmlGetProp(pNode, (const xmlChar*)"col2");
      xmlChar *pCol3 = xmlGetProp(pNode, (const xmlChar*)"col3");
      xmlChar *pCol4 = xmlGetProp(pNode, (const xmlChar*)"col4");
      if (NULL == pRowId || NULL == pCol1 || NULL == pCol2 || NULL == pCol3 || NULL == pCol4) {
        // TODO: output error
        return;
      }
      int32_t rowIdx = ((char*)pRowId)[0] - '0';
      double col1Value = atof((const char*)pCol1);
      double col2Value = atof((const char*)pCol2);
      double col3Value = atof((const char*)pCol3);
      double col4Value = atof((const char*)pCol4);
      switch (rowIdx) {
      case 0:
        matrix.row1[0] = col1Value;
        matrix.row1[1] = col2Value;
        matrix.row1[2] = col3Value;
        matrix.row1[3] = col4Value;
        break;
      case 1:
        matrix.row2[0] = col1Value;
        matrix.row2[1] = col2Value;
        matrix.row2[2] = col3Value;
        matrix.row2[3] = col4Value;
        break;
      case 2:
        matrix.row3[0] = col1Value;
        matrix.row3[1] = col2Value;
        matrix.row3[2] = col3Value;
        matrix.row3[3] = col4Value;
        break;
      case 3:
        matrix.row4[0] = col1Value;
        matrix.row4[1] = col2Value;
        matrix.row4[2] = col3Value;
        matrix.row4[3] = col4Value;
        break;
      default:
        break;
      }
      xmlFree(pRowId);
      xmlFree(pCol1);
      xmlFree(pCol2);
      xmlFree(pCol3);
      xmlFree(pCol4);
    }
    pNode = pNode->next;
  }
  return;
}

void XmlParser::ParseAxis(xmlDocPtr pDoc, xmlNodePtr pNode, XmlDvrkAxis &axis)
{
  xmlChar *pId = xmlGetProp(pNode, (const xmlChar*)"id");
  xmlChar *pBoardId = xmlGetProp(pNode, (const xmlChar*)"board");
  xmlChar *pPortId = xmlGetProp(pNode, (const xmlChar*)"port");
  xmlChar *pAmpMask = xmlGetProp(pNode, (const xmlChar*)"ampmask");
  if (NULL == pId || NULL == pBoardId || NULL == pPortId || NULL ==  pAmpMask) {
    // TODO: output error
    return;
  }
  int32_t id = ((char*)pId)[0] - '0';
  int32_t boardId = ((char*)pBoardId)[0] - '0';
  int32_t portId = ((char*)pPortId)[0] - '0';
  int8_t ampmask = ((char*)pAmpMask)[0] - '0';
  axis.id = id;
  axis.board = boardId;
  axis.port = portId;
  axis.ampmask = ampmask;

  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Analog")) {
      ParseAnalog(pDoc, pNode, axis.analog);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Encoder")) {
      ParseEncoder(pDoc, pNode, axis.encoder);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Drive")) {
      ParseDrive(pDoc, pNode, axis.drive);
    }
    pNode = pNode->next;
  }

  xmlFree(pBoardId);
  xmlFree(pPortId);
}

void XmlParser::ParseAnalog(xmlDocPtr pDoc, xmlNodePtr pNode, XmlAnalog &analog)
{
  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Min")) {
      ParseInt(pDoc, pNode, &analog.min);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Max")) {
      ParseInt(pDoc, pNode, &analog.max);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Zero")) {
      ParseInt(pDoc, pNode, &analog.zero);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Scale")) {
      ParseInt(pDoc, pNode, &analog.scale);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseEncoder(xmlDocPtr pDoc, xmlNodePtr pNode, XmlEncoder &encoder)
{
  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"BitsToPosSI")) {
      ParseDouble(pDoc, pNode, &encoder.BitsToPosSI);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseDrive(xmlDocPtr pDoc, xmlNodePtr pNode, XmlDrive &drive)
{
  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"MaxAmps")) {
      ParseDouble(pDoc, pNode, &drive.maxAmps);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"AmpsToBits")) {
      ParseAmpsToBits(pDoc, pNode, drive.ampsToBits);
    }
    pNode = pNode->next;
  }
}

void XmlParser::ParseAmpsToBits(xmlDocPtr pDoc, xmlNodePtr pNode, XmlAmpsToBits &ampsToBits)
{
  pNode = pNode->xmlChildrenNode;
  while (NULL != pNode) {
    if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Offset")) {
      ParseInt(pDoc, pNode, &ampsToBits.offset);
    } else if (0 == xmlStrcmp(pNode->name, (const xmlChar*)"Scale")) {
      ParseDouble(pDoc, pNode, &ampsToBits.scale);
    }
    pNode = pNode->next;
  }
}

int XmlParser::ParseInt(xmlDocPtr pDoc, xmlNodePtr pNode, int* pValue)
{
  xmlChar* pValueStr = xmlNodeGetContent(pNode);

  if (NULL != pValueStr)
  {
    int iStrLen = (int)strlen((const char*)pValueStr);
    if (iStrLen < 11)
      *pValue = atoi((const char*)pValueStr);
    else
      *pValue = 0 ;

    xmlFree(pValueStr);
  }

  return 0 ;
}

int XmlParser::ParseDouble(xmlDocPtr pDoc, xmlNodePtr pNode, double* pValue)
{
  xmlChar* pValueStr = xmlNodeGetContent(pNode);

  if (NULL != pValueStr)
  {
    int iStrLen = (int)strlen((const char*)pValueStr);
    if (iStrLen > 0)
      *pValue = atof((const char*)pValueStr);
    else
      *pValue = 0 ;

    xmlFree(pValueStr);
  }

  return 0 ;
}

int XmlParser::ParseFloat(xmlDocPtr pDoc, xmlNodePtr pNode, float* pValue)
{
  char *endptr;
  xmlChar* pValueStr = xmlNodeGetContent(pNode);

  if (NULL != pValueStr)
  {
    errno = 0;
    *pValue = strtof((const char*)pValueStr, &endptr);
    if ((errno == ERANGE && (*pValue == LONG_MAX || *pValue == LONG_MIN)) || (errno != 0 && *pValue == 0)) {
      perror("strtol");
      exit(EXIT_FAILURE);
    }
    if (endptr == (const char*)pValueStr) {
      fprintf(stderr, "No digits were found\n");
      exit(EXIT_FAILURE);
    }

    xmlFree(pValueStr);
  }

  return 0 ;
}

