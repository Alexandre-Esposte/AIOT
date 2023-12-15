
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "model.h"

#define TAM 10
#define TAM2 20
Adafruit_MPU6050 mpu;


float STD(const float* vetor, size_t tamanho) {
    // Verifica se o vetor está vazio
    if (tamanho == 0) {
        Serial.println("Erro: O vetor está vazio.");
        return 0.0;  // Retorno padrão em caso de vetor vazio
    }

    // Calcula a média dos valores no vetor
    float soma = 0.0;
    for (size_t i = 0; i < tamanho; ++i) {
        soma += vetor[i];
    }
    float media = soma / tamanho;

    // Calcula a soma dos quadrados das diferenças em relação à média
    float somaQuadradosDiferencas = 0.0;
    for (size_t i = 0; i < tamanho; ++i) {
        float diferenca = vetor[i] - media;
        somaQuadradosDiferencas += diferenca * diferenca;
    }

    // Calcula e retorna o desvio padrão
    return sqrt(somaQuadradosDiferencas / tamanho);
}
float Media(const float* vetor, size_t tamanho) {
    // Verifica se o vetor está vazio
    if (tamanho == 0) {
        Serial.println("Erro: O vetor está vazio.");
        return 0.0;  // Retorno padrão em caso de vetor vazio
    }

    // Calcula a soma dos valores no vetor
    float soma = 0.0;
    for (size_t i = 0; i < tamanho; ++i) {
        soma += vetor[i];
    }

    // Calcula e retorna a média
    return soma / tamanho;
}
float Maximo(const float* vetor, size_t tamanho) {
    // Verifica se o vetor está vazio
    if (tamanho == 0) {
        Serial.println("Erro: O vetor está vazio.");
        return 0.0;  // Retorno padrão em caso de vetor vazio
    }

    // Inicializa o valor máximo com o primeiro elemento do vetor
    float valorMaximo = vetor[0];

    // Encontra o valor máximo no vetor
    for (size_t i = 1; i < tamanho; ++i) {
        if (vetor[i] > valorMaximo) {
            valorMaximo = vetor[i];
        }
    }

    return valorMaximo;
}
float Minimo(const float* vetor, size_t tamanho) {
    // Verifica se o vetor está vazio
    if (tamanho == 0) {
        Serial.println("Erro: O vetor está vazio.");
        return 0.0;  // Retorno padrão em caso de vetor vazio
    }

    // Inicializa o valor mínimo com o primeiro elemento do vetor
    float valorMinimo = vetor[0];

    // Encontra o valor mínimo no vetor
    for (size_t i = 1; i < tamanho; ++i) {
        if (vetor[i] < valorMinimo) {
            valorMinimo = vetor[i];
        }
    }

    return valorMinimo;
}

int encontrarMaisFrequente(const int* array, size_t tamanho) {
    // Verifica se o array está vazio
    if (tamanho == 0) {
        Serial.println("Erro: O array está vazio.");
        return 0;  // Retorno padrão em caso de array vazio
    }

    // Usa arrays para contar a frequência de cada número
    int frequencia[4] = {0};  // Inicializa todos os elementos com zero

    for (size_t i = 0; i < tamanho; ++i) {
        int valor = array[i];
        if (valor >= 0 && valor <= 3) {
            frequencia[valor]++;
        }
    }

    // Encontra o valor mais frequente
    int valorMaisFrequente = 0;
    int maiorFrequencia = 0;

    for (int i = 0; i < 4; ++i) {
        if (frequencia[i] > maiorFrequencia) {
            valorMaisFrequente = i;
            maiorFrequencia = frequencia[i];
        }
    }

    // Verifica se há um valor mais frequente
    if (maiorFrequencia > 1) {
        return valorMaisFrequente;
    } else {
        return 4;  // Se não houver valor mais frequente, retorna zero
    }
}


void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  while (!model.begin()) {
        Serial.print("Erro ao carregar o modelo: ");
        Serial.println(model.getErrorMessage());
    }

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float vars[6],ax[TAM],ay[TAM],az[TAM],gx[TAM],gy[TAM],gz[TAM], to_pred[24];
  float ax_mean, ay_mean, az_mean, gx_mean, gy_mean, gz_mean;
  float ax_std, ay_std, az_std, gx_std, gy_std, gz_std, ax_max;
  float ay_max, az_max, gx_max, gy_max, gz_max, ax_min, ay_min;
  float az_min, gx_min, gy_min, gz_min;
  int i ;
  int persistencia[TAM2];
  vars[0]= a.acceleration.x;
  vars[1] = a.acceleration.y;
  vars[2]= a.acceleration.z;
  vars[3] = g.gyro.x;
  vars[4] = g.gyro.y;
  vars[5] = g.gyro.z;
  for(int j =0; j<TAM2; j++)
  {
    for(int i = 0 ; i< TAM; i++)
    {
      mpu.getEvent(&a, &g, &temp);

      ax[i] = a.acceleration.x;
      ay[i] = a.acceleration.y;
      az[i] = a.acceleration.z;
      gx[i] = g.gyro.x;
      gy[i] = g.gyro.y;
      gz[i] = g.gyro.z;

      delay(10);
    }
    ax_mean = Media(ax,TAM);
    ax_std =  STD(ax,TAM);
    ax_max =  Maximo(ax,TAM);  
    ax_min =  Minimo(ax,TAM);

    ay_mean = Media(ay,TAM);
    ay_std =  STD(ay,TAM);
    ay_max =  Maximo(ay,TAM);
    ay_min =  Minimo(ay,TAM);

    az_mean = Media(az,TAM);
    az_std =  STD(az,TAM);
    az_max =  Maximo(az,TAM);
    az_min =  Minimo(az,TAM);

    gx_mean = Media(gx,TAM);
    gx_std =  STD(gx,TAM);
    gx_max =  Maximo(gx,TAM);
    gx_min =  Minimo(gx,TAM);

    gy_mean = Media(gy,TAM);
    gy_std =  STD(gy,TAM);
    gy_max =  Maximo(gy,TAM);
    gy_min =  Minimo(gy,TAM);

    gz_mean = Media(gz,TAM);
    gz_std =  STD(gz,TAM);
    gz_max =  Maximo(gz,TAM);
    gz_min =  Minimo(gz,TAM);

    to_pred[0] = ax_mean;
    to_pred[1] = ay_mean;
    to_pred[2] = az_mean;
    to_pred[3] = gx_mean;
    to_pred[4] = gy_mean;
    to_pred[5] = gz_mean;
    to_pred[6] = ax_std;
    to_pred[7] = ay_std;
    to_pred[8] = az_std;
    to_pred[9] = gx_std;
    to_pred[10] = gy_std;
    to_pred[11] = gz_std;
    to_pred[12] = ax_max;
    to_pred[13] = ay_max;
    to_pred[14] = az_max;
    to_pred[15] = gx_max;
    to_pred[16] = gy_max;
    to_pred[17] = gz_max;
    to_pred[18] = ax_min;
    to_pred[19] = ay_min;
    to_pred[20] = az_min;
    to_pred[21] = gx_min;
    to_pred[22] = gy_min;
    to_pred[23] = gz_min;

    persistencia[j] = model.predictClass(to_pred);

  }
  
  int pred = encontrarMaisFrequente(persistencia,TAM2);

  if(pred == 0)
    Serial.println("Desligado");
  else if(pred == 1)
    Serial.println("Potencia 1");
  else if(pred == 2)
    Serial.println("Potencia 2");
  else if(pred == 3)
    Serial.println("Potencia 3");
  else
    Serial.println("Error");

}
