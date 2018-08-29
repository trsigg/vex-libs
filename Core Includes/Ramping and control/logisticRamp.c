typedef struct {
  float k, M, C;
} logisticRamper;

void initializeLogisticRamp(logisticRamper *ramper, float k=1, float M=1, float intercept = 0.25) {
  ramper->k = k;
  ramper->M = M;
  ramper->C = M / intercept - 1;
}

float logisticRampRuntime(logisticRamper *ramper, float input) {
  return ramper->M / (1 + ramper->C * exp(-ramper->k * ramper->M * input));
}
