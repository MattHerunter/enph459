import processing.serial.*;

Serial arduinoPort;
String filePrefix, portName, fileString, val;
Table table;

int[] data;
int numTests;
boolean readFlag, tableInitializedFlag;


void setup() {
  portName = Serial.list()[0];
  arduinoPort = new Serial(this, portName, 128000); //set up your port to listen to the serial port 
  filePrefix = "fdc4000rpm4270";
  val = "";
  table = new Table();
  numTests = 0;
  readFlag = false;
  tableInitializedFlag = false;
}

void draw()
{
  while (arduinoPort.available() > 0) {
    // 10 is ASCII for newline
    val = arduinoPort.readStringUntil(10);
    //println(val);
    // For some reason the serial port sometimes has long strings of null characters appearing on it
    if (val != null) {
      if (val.equals("Begin\n")) {
        println("In begin");
        readFlag = true;
      } else if (val.equals("End\n") && readFlag) {
        println("In end");
        fileString = filePrefix + "_" + str(numTests) + "_" +  str(hour())  + "_" + str(minute()) + "_" + str(second()) + ".csv"; //this filename is of the form year+month+day+readingCounter
        saveTable(table, fileString);
        table.clearRows();
        numTests++;
        readFlag = false;
      } else if (readFlag) {
        //println("In write");
        data = int(split(val.trim(), ','));
        // Set the number of columns in the table to the number of columns in the received data
        if (!tableInitializedFlag) {
          for (int ii = 0; ii < data.length; ii++) {
            table.addColumn("atd"+str(ii));
          }
          tableInitializedFlag = true;
        }

        TableRow newRow = table.addRow(); // Add a row for this new reading
        for (int ii = 0; ii < data.length; ii++) {
          newRow.setInt("atd"+str(ii), data[ii]);
        }
      }
    }
  }
}