/*
   Node C
*/

int[] candidates;
int uID;
String[] nodes = {"A", "B", "C"};
char myID = "C";
int right = 1;
int left = 2;
int rooms[] = {1, 2};
String[][] routings = {(room1, "A"), (room2, "B")};
String[][] directions = {("A", -1, -1, 0), ("B", -1, -1, 0)};

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  String[] recvd = recv();
  if (recvd[0].contains("START")) {
    sendDist(recvd[3]); // send distance to user to all other nodes
    ID = recvd[1];
  }
  else if (recvd[0].contains("CANDIDATE")) {
    int d = computeDist();
    if (recvd[4] > d) {
      send("SUPERIOR", recvd[2], myID, uID, d);
    }
  }
  else if (recvd[0].contains("SUPERIOR") {

  }
}

int sendDist(int userID) {
  int d = computeDist();
  for (int i = 0; i < nodes.length; i++) {
    if (node[i] != myID) {
      // send(TO, FROM, USERID, DISTANCE)
      send("CANDIDATE", node[i], myID, userID, d);
    }
  }
}

int send(String msgType, char toNode, char thisID, int uID, int d) {

}

String[] recv() {
  // continuously read for incoming msgs
  // Format of output: 
  // String MSGTYPE, char myID, char fromID, int userID, int distance
}

int computeDist() {
  // check distance to user
}
}

