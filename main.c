#include <stdio.h>
#include <immintrin.h>
#include <stdlib.h>
#include <time.h>

#define SIZE 1<<16

typedef struct {
    float x;
    float y;
} Vec2;

typedef struct {
    float x[];
    float y[];
} Vec2SOA;

void VectorizedSumOfSquares(const float *A, const float *B, float *results, int size) {
    int i;
    for (i = 0; i + 8 <= size; i += 8) {
        __m256 vecA = _mm256_load_ps(&A[i]);
        vecA = _mm256_mul_ps(vecA, vecA);

        __m256 vecB = _mm256_load_ps(&B[i]);
        vecB = _mm256_mul_ps(vecB, vecB);

        __m256 result = _mm256_add_ps(vecA, vecB);
        _mm256_store_ps(&results[i], result);
    }

    for (; i < size; i++) {
        results[i] = A[i] * A[i] + B[i] * B[i];
    }
}

void SumOfSquares(const float *A, const float *B, float *results, int size) {
    for (int i = 0; i < size; ++i) {
        results[i] = A[i] * A[i] + B[i] * B[i];
    }
}

//Array of structs version
void VectorizedRotate(Vec2 *offsets, size_t size) {
    size_t i;
    for (i = 0; i + 8 <= size; i += 8) {
        __m256 vectorX = _mm256_loadu_ps(&offsets[i].x);
        __m256 vectorY = _mm256_loadu_ps(&offsets[i].y);
        __m256 temp = vectorX;

        __m256 mask = _mm256_set1_ps(-0.0f);

        vectorX = vectorY;
        vectorY = _mm256_xor_ps(temp, mask); //flip sign
        _mm256_storeu_ps(&offsets[i].x, vectorX);
        _mm256_storeu_ps(&offsets[i].y, vectorY);
    }
}

//Struct of Array Version
void VectorizedRotate(Vec2SOA *vectors, size_t size) {
    size_t i;
    for (i = 0; i + 8 <= size; i += 8) {
        __m256 vectorX = _mm256_loadu_ps(&vectors->x[i]);
        __m256 vectorY = _mm256_loadu_ps(&vectors->y[i]);
        __m256 temp = vectorX;

        __m256 mask = _mm256_set1_ps(-0.0f);

        vectorX = vectorY;
        vectorY = _mm256_xor_ps(temp, mask);
        _mm256_storeu_ps(&vectors->x[i], vectorX);
        _mm256_storeu_ps(&vectors->y[i], vectorY);
    }
}

int main() {
    float A[] __attribute__((aligned(32))) = {1, 2, 3, 4, 5, 6, 7, 8};
    float B[] __attribute__((aligned(32))) = {8, 9, 11, 12, 13, 14, 15, 16};
    float result[SIZE];

    time_t start = clock();
    VectorizedSumOfSquares(A, B, result, SIZE);
    time_t end = clock();
    printf("Vector Time: %lf\n", (double) (end - start) / CLOCKS_PER_SEC);

    SumOfSquares(A, B, result, SIZE);

    for (int i = 0; i < SIZE; ++i) {
        printf("%f ", result[i]);
    }

    Vec2 *offsets[SIZE];


    return 0;
}
