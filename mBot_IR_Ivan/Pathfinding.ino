/*

Pathfinding module for mBot_IR_Ivan

*/

void robotRunPathFindingLoop()
{
  clearIRBufferAndCheckCancelProgram();

  distance = UltrasonicSensor.distanceCm();

  // Sensor throws in random zeroes every once in a while, need to ignore those
  if ((distance != 0) && (distance < 25))
  {
    TurnRight();
  }
  else
  {
    Forward();
  }
  
	delay(100);
}
