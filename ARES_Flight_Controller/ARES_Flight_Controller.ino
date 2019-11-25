#define MID_DESCENT_ALT //If below this altitude, descend at medium speed
#define SLOW_DESCENT_ALT //If below this altitude, descend at low speed
#define LANDING_ALT //If below this altitude, turn into wind and land

int phase = 0;
  // phase0 pre-deployment
  // phase1 deployed & waiting for GPS lock (slow circles)
  // phase2 detecting wind direction (slow circles)
  // phase3 flight to target (straight line flight)
  // phase4 fast descent (tight circles)
  // phase5 medium descent (medium circles)
  // phase6 slow descent (wide circles)
  // phase7 landing into wind (straight-line landing)

void setup() {
  setupSensors();
  
}

void loop() {
  updateSensors();
  updatePhase();
  switch(phase) {
    case 0:
      //behavior before deployment. perhaps just checking GPS and listening to dead man switch
      break;
    case 1:
      //behavior after deployment, slow circles while waiting for GPS lock
      break;
    case 2:
      //behavior while detecting wind direction.  tracking GPS ground speed as well as heading
      break;
    case 3:
      //behavior while flying to target
      break;
    case 4:
      //behavior while descending quickly. top of wedding cake
      break;
    case 5:
      //behavior while mid-cake
      break;
    case 6:
      //behavior at bottom of cake
      break;
    case 7:
      //turn towards wind and flare to land
      break;
  }
}
