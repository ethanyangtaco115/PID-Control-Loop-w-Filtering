class PID {
  public:
    float p, i, d, maxOutput;
    
    PID(float p, float i, float d, float maxOutput) {
      this->p = p;
      this->i = i;
      this->d = d;
      this->maxOutput = maxOutput;
    }
};

PID pid(0, 0, 0, 0); // Placeholder values,replace with PID gains and max output

struct PIDState {
  unsigned long previousTime;
  float lastError;
  float error_int;
  
  // filter vars
  static const int WINDOW_SIZE = 5;
  float errorHistory[WINDOW_SIZE];
  int errorIndex;

  // intialize values
  PIDState() {
    previousTime = 0;
    lastError = 0;
    error_int = 0;
    errorIndex = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
      errorHistory[i] = 0;
    }
  }
};

// intialize PID states
PIDState pidX;
PIDState pidY;

// filter coefficients
const float SG_COEFFS[5] = {-3, -2, 0, 2, 3};
const float SG_DENOM = 10.0;

void setup() {
  Serial.begin(115200);
}

float filterDerivative(PIDState &state, float error, float dt) {
  state.errorHistory[state.errorIndex] = error;
  state.errorIndex = (state.errorIndex + 1) % PIDState::WINDOW_SIZE;

  float weightedSum = 0;
  int idx = state.errorIndex;
  for (int i = 0; i < PIDState::WINDOW_SIZE; i++) {
    weightedSum += SG_COEFFS[i] * state.errorHistory[idx];
    idx = (idx + 1) % PIDState::WINDOW_SIZE;
  }

  return (weightedSum / (SG_DENOM * dt));
}

float pidCalculate(float input, float setPoint, PIDState &state) {
  unsigned long currentTime = micros();
  float deltaTime = (currentTime - state.previousTime) / 1000000.0;
  if (deltaTime <= 0) deltaTime = 1e-6;

  float error = setPoint - input;
  state.error_int += error * deltaTime;

  float error_der = filterDerivative(state, error, deltaTime);

  float output = pid.p * error + pid.i * state.error_int + pid.d * error_der;
  output = constrain(output, -pid.maxOutput, pid.maxOutput);

  state.lastError = error;
  state.previousTime = currentTime;

  return output;
}

void loop() {
  float inputX = 0; // Replace with actual sensor data
  float inputY = 0;
  float setPointX = 0;
  float setPointY = 0;

  float outputX = pidCalculate(inputX, setPointX, pidX);
  float outputY = pidCalculate(inputY, setPointY, pidY);

  Serial.print("Output X: "); //debug output
  Serial.println(outputX);
  Serial.print("Output Y: ");
  Serial.println(outputY);

  delay(10);
}
