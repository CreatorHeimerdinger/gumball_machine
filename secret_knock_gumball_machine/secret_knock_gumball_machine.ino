#include <Servo.h> 
/*
 Analog Pin 0: Piezo speaker (connected to ground with 480K ohm pulldown resistor)
 Analog Pin 1: 10K Trim pot.
 Digital Pin 2: Switch to enter new knock code.
 Digital Pin 3: Success LED. 
 Digital Pin 4: Failure LED. 
 Digital Pin 5: Activity light 1. (Optional: lights up when the servo is turning.) 
 Digital Pin 6: Activity light 2. (Optional: lights up the gumball end point)
 Digital Pin 7: Servo motor signal.
 */
#define VAL_REF 500
#define programSwitch A5
// Pin definitions
const int knockSensor = 0;         // Piezo sensor on analog pin 0.
const int trimPot = 1;             // Trim potentiometer that allows the user to set the sensitivity
//const int programSwitch = 2;       // When this is high we program a new code
const int servoPin = 7;            // Servo that we're controlling. 
const int redLED = 4;              // Status LED for failed knock
const int greenLED = 3;            // Status LED for successful knock.
const int activityLED1 = 5;        // Lights up the servo when its working. (Optional)
const int activityLED2 = 6;        // Gumball tray light. (Optional)

// Tuning constants.  Could be made vars and hooked to potentiometers for soft configuration, etc.
const int thresholdMax = 1000;       // Maximum value we can set for the knock sensitivity based on the trim pot.  Increase this if it's too sensitive.
const int rejectValue = 30;        // If an individual knock is off by this percentage of a knock we ignore. (30 is pretty lose. 10 is strict)
const int averageRejectValue = 20; // If the average timing of the knocks is off by this percent we ignore. (20 is pretty lose, 10 is strict.)
const int debounceThreshold = 80;  // Simple debounce timer to make sure we don't register more than one knock.

const int maximumKnocks = 20;       // Maximum number of knocks to listen for.
const int knockComplete = 1500;     // Longest time to wait for a knock before we assume that it's finished.

// Variables.
Servo gumballServo;                  // The servo that dispenses a gumball
int secretCode[maximumKnocks] = {
  50, 25, 25, 50, 100, 50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // Initial setup: "Shave and a Hair Cut, two bits." 100=full note, 50=half note, 25=quarter note, etc.
int knockReadings[maximumKnocks];    // When someone knocks this array fills with delays between knocks. (A correct knock looks a lot like the line above).
int knockSensorValue = 0;            // Most recent reading of the knock sensor.
int threshold =1023;                 // Minimum signal from the piezo to register as a knock.  (Set by the trim pot.)
int programButtonPressed = false;    // Flag so we remember the programming button setting at the end of the cycle.
long counter = 0;                    // Used to time the display light and the trim pot readings.

void setup() 
{
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(activityLED1, OUTPUT);
  pinMode(activityLED2, OUTPUT);

  pinMode(programSwitch, INPUT_PULLUP);
  //digitalWrite(A5, HIGH);

  threshold = map(VAL_REF, 0, 1023, 0, thresholdMax);  // Initial reading of the trim pot.
  //threshold = map(analogRead(trimPot), 0, 1023, 0, thresholdMax);

  digitalWrite(redLED, HIGH);      // Turn on all the lights so we can be sure they work.
  digitalWrite(greenLED, HIGH);
  digitalWrite(activityLED1, HIGH);
  digitalWrite(activityLED2, HIGH);

  gumballServo.attach(servoPin);   // Initialize the servo.
  delay(50);
  for (int i=0;i<150;i++) // Reset the rotation of the servo.
  {        
    gumballServo.write(0);         // Write the location a few times just in case the wheel is a long way off or binding somewhere.
    delay(20);
  }  
  delay(50);
  gumballServo.detach();           // Detach the servo when not using it.

  digitalWrite(redLED, LOW);       // Turn off the lights, testing is over.
  digitalWrite(greenLED, LOW);
  digitalWrite(activityLED1, LOW);
  digitalWrite(activityLED2, LOW);

  Serial.begin(9600);               // To debug search and replace (Ctl-F) "Serial." with "Serial." 
  Serial.println("Program start."); // NOTE: This will reduce the sensitivity of the knock sensor.

}

void loop() 
{
  // Listen for any knock at all.
  knockSensorValue = analogRead(knockSensor);
  if ( knockSensorValue>512) knockSensorValue=10;
  else  knockSensorValue=999;
  counter++;
  if (counter>=1500)
  {
    counter=0;
    digitalWrite(activityLED2, LOW);       // Turn off the tray light after a while.  If it's on.
  }

  threshold = map(VAL_REF, 0, 1023, 0, thresholdMax);  // Read the trim pot to see what our lower limit is.
  //threshold = map(analogRead(trimPot), 0, 1023, 0, thresholdMax);
/*
  Serial.print("k:");					// These four lines are for debugging only.  Feel free to remove them or comment them out.
  Serial.print(knockSensorValue);                     // Displays something this in the serial monitor: "k:0 t:6" where "k" is the value of the knock sensor
  Serial.print(" t:");                                // and "t" is the value of the trim pot.  When k is higher than p a knock is detected.
  Serial.println(threshold);
*/
  if (digitalRead(programSwitch)==LOW) // Is the program button pressed?
  {  
    programButtonPressed = true;           // Yes, so lets save that state.
    digitalWrite(redLED, HIGH);            // ...and turn on the the lights so we know we're programming.
    digitalWrite(greenLED, HIGH);
  } 
  else // Otherwise reset all the ... everything.
  {
    programButtonPressed = false;          
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
  }
  if (knockSensorValue > threshold)    listenToSecretKnock();
} 

// Records the timing of knocks.
void listenToSecretKnock()
{
  int i = 0;
  int counter = 0;
  int knock_state=0;
  int thresholdUp=0;
  int thresholdLow=0;
  Serial.println("(knock started)");
   
  digitalWrite(activityLED2, LOW);    // Turn off any other lights that might be on.

  // First lets reset the listening array.
  for (i=0;i<maximumKnocks;i++)  knockReadings[i]=0;

  int currentKnockNumber=0;         	         // Increment for the array.
  int startTime=millis();           	         // Reference for when this knock started.
  int now=millis();

  digitalWrite(greenLED, HIGH);      	        // Light the green LED for knock feedback.

  delay(debounceThreshold);
  digitalWrite(greenLED, LOW); 

  do 
  {
    // Listen for the next knock or wait for it to time out. 
    knockSensorValue = analogRead(knockSensor);
    //threshold = map(analogRead(trimPot), 0, 1023, 0, thresholdMax);
    threshold = map(VAL_REF, 0, 1023, 0, thresholdMax);
    thresholdUp=threshold+threshold/3;
    thresholdLow=threshold-threshold/3;
    /*
    Serial.print("p:");
    Serial.print(knockSensorValue);
    Serial.print(" t:"); 
    Serial.println(threshold);
    */
    if (knock_state==0 && knockSensorValue > thresholdUp)// Got another knock...
    {                   
      knock_state=1;
      Serial.println(" knock!");
      now=millis();                                      // Record the delay time.
      knockReadings[currentKnockNumber] = now-startTime;
      Serial.println(knockReadings[currentKnockNumber]);
      currentKnockNumber ++;                             // Increment the counter.
      startTime=now;          
      digitalWrite(greenLED, HIGH);      	         // Light the knock LED.

      delay(debounceThreshold/2);                        // Debounce the knock sensor. (And turn the LED off halfway through.)
      digitalWrite(greenLED, LOW);  
      delay(debounceThreshold/2);   
    }
    else
    if(knock_state==1 && knockSensorValue < thresholdLow)
    {
      knock_state=0;
    }

    now=millis();

    // Did we timeout or have too many knocks?
  } 
  while ((now-startTime < knockComplete) && (currentKnockNumber < maximumKnocks));

  // We have a completed knock, lets see if it's valid
  if (programButtonPressed==false)
  {  
    if (validateKnock() == true) triggerSuccessfulAction(); 
    else triggerFailedAction(); 
  } 
  else  // If we're in programming mode we still run it through the validate function in order to get some useful numbers.
  { 
    validateKnock();
    // Blink the green and red lights in sequence to show that program is complete.
    Serial.println("New lock stored.");
    digitalWrite(redLED, LOW);
    digitalWrite(greenLED, LOW);
    for (i=0;i<3;i++)
    {
      delay(100);
      digitalWrite(redLED, HIGH);
      digitalWrite(greenLED, HIGH);
      delay(100);
      digitalWrite(redLED, LOW);
      digitalWrite(greenLED, LOW);      
    }
  }
}

// We got a good knock, so do something!
void triggerSuccessfulAction()
{
  Serial.println("Success!");
  int i=0;

  digitalWrite(greenLED, HIGH);       // Turn on this light so the user knows the knock was successful.  (Important if the optional status lights are missing.)
  digitalWrite(activityLED1, HIGH);   // Turn on the servo light to show where the action is.

  gumballServo.attach(servoPin);      // Now we start with the servo.
  delay(50);
  int lightStatus=HIGH;               // Set this so we can blink the LED as the servo cycles.
  int d=10;
  for (int i=0;i<=180;i++)
  {        
    if (i>140) gumballServo.write(140);   // Limit the rotation of the servo to 140 degrees.  We write this a few times in case it had trouble getting there.
    else gumballServo.write(i);       // Tell servo to rotate to position in variable 'i'.
    if (i%18==0) lightStatus =! lightStatus;                 // Change the light state every 18 degrees. (AKA: blink 5 times)
    digitalWrite(activityLED1, lightStatus); 
    delay(d);
  }
  delay(100);   
  digitalWrite(activityLED1, LOW);   

  digitalWrite(activityLED2, HIGH);  // Light up the gumball tray light.
  for (int i=140;i>=0;i--) // Return the servo to it's start location.
  {         
    gumballServo.write(i);   
    delay(d);
  }   
  delay(50);
  gumballServo.detach();             // Done with the servo, so detach it.

  delay(100);
}

// We didn't like the knock.  Indicate displeasure.
void triggerFailedAction()
{
  Serial.println("Secret knock failed.");
  digitalWrite(greenLED, LOW);  
  digitalWrite(redLED, LOW);		
  for (int i=0;i<4;i++)
  {					
    digitalWrite(redLED, HIGH);
    delay(100);
    digitalWrite(redLED, LOW);
    delay(100);
  }
}

// Checks if our knock matches the secret.
// Returns true if it's a good knock, false if it's not.
boolean validateKnock()
{
  int i=0;

  // Simplest check first: Did we get the right number of knocks?
  int currentKnockCount = 0;
  int secretKnockCount = 0;
  int maxKnockInterval = 0;             // We use this later to normalize the times.

  for (i=0;i<maximumKnocks;i++)
  {
    if (knockReadings[i] > 0)  currentKnockCount++;
    if (secretCode[i] > 0)  secretKnockCount++;
    if (knockReadings[i] > maxKnockInterval)  maxKnockInterval = knockReadings[i];	// Collect normalization data while we're looping.   
  }

  // If we're recording a new knock, save the relevant info and get out of here.
  if (programButtonPressed==true)
  {
     // Normalize the knock timing
    for (i=0;i<maximumKnocks;i++) secretCode[i]= map(knockReadings[i],0, maxKnockInterval, 0, 100);
    // And flash the lights in the recorded pattern to let us know it's been programmed.
    digitalWrite(greenLED, LOW);
    digitalWrite(redLED, LOW);
    delay(750);
    //Start playing back the knocks
    digitalWrite(greenLED, HIGH);
    digitalWrite(redLED, HIGH);  // First knock
    delay(40);
    for (i = 0; i < maximumKnocks ; i++)
    {
      digitalWrite(greenLED, LOW);
      digitalWrite(redLED, LOW);  
      if (programButtonPressed==true)
      {  // Only turn it on if there's a delay
        if (secretCode[i] > 0)
        {                                   
          delay(map(secretCode[i],0, 100, 0, maxKnockInterval)); // Expand the time back out to what it was.  Roughly. 
          digitalWrite(greenLED, HIGH);
          digitalWrite(redLED, HIGH);  
        }
      }
      delay(40);
      digitalWrite(greenLED, LOW);
      digitalWrite(redLED, LOW);  
    }
    return false; 	// We don't do anything when we are recording a new knock.
  }

  if (currentKnockCount != secretKnockCount)return false;   // Return false if the number of knocks are wrong.
  /*  Now we compare the relative intervals of our knocks, not the absolute time between them.
   (ie: if you do the same pattern slow or fast it should still work.)
   This makes it less picky, which does make it less secure but also makes it
   less of a pain to use if you're tempo is a little slow or fast. 
   */
  /*
  boolean codeFound=true;
  int totaltimeDifferences=0;
  int timeDiff=0;

  for (i=0;i<maximumKnocks;i++)
  { // Normalize the times
    knockReadings[i]= map(knockReadings[i],0, maxKnockInterval, 0, 100);      
    timeDiff = abs(knockReadings[i]-secretCode[i]);
    if (timeDiff > rejectValue){ // Individual value too far out of whack
      codeFound=false;
    }
    totaltimeDifferences += timeDiff;
  }
  // It can also fail if the whole thing is too inaccurate.
  if (totaltimeDifferences/secretKnockCount>averageRejectValue) codeFound = false;
  */
  /* bgt's version*/
  boolean codeFound=true;
  double knck2=0.0,sec2=0.0,dot=0.0,p=0.0,temp=0;
  for (i=0;i<maximumKnocks;i++)
  { // Normalize the times
    knockReadings[i]= map(knockReadings[i],0, maxKnockInterval, 0, 100);      
  }
  for(i=0; i<maximumKnocks; i++)
  {
    temp=knockReadings[i]*knockReadings[i];
    knck2+=temp;
    temp=secretCode[i]*secretCode[i];
    sec2+=temp;
    temp=secretCode[i]*knockReadings[i];
    dot+=temp;
  }
  p=dot/(sqrt(sec2)*sqrt(knck2));
  if(p>0.97) codeFound=true;
  else codeFound=false;
  Serial.print("P:"); // NOTE: This will reduce the sensitivity of the knock sensor.
  Serial.println(p); // NOTE: This will reduce the sensitivity of the knock sensor.
  
  if (codeFound==false) return false;
  else return true;
}



