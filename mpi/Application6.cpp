#include <iostream>
#include <vector>
#include <random>
#include "mpi.h"

double heavyfunc(double init) {
    int n = 5'000;
    double r = 0;
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            r += cos((double)i + init) * sin(init*(double)j) * pow((double)i / 100, 1.23123);
        }
    }
    return r;
}

std::pair<int, int> iter(int seed) {
    std::mt19937 gen(seed);
    std::uniform_real_distribution<>dx(-1, 1);
    int n = 0;
    int nc = 0;
    while (n < 1e6 / 3) {
        double x = dx(gen);
        double y = dx(gen);
        if (x * x + y * y < 1) {
            nc++;
        }
        n++;
    }
    return { n, nc };
}

//--------find pi

//int main()
//{
//    MPI_Init(nullptr, nullptr);
//    
//    int rank, mpi_size;
//    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
//    MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);
//    int n = 0;
//    int nc = 0;
//    if (rank == 0) {
//        for (int i = 1; i < mpi_size; ++i) {
//            int input = i;
//            MPI_Send(&input, 1, MPI_INT, i, 0, MPI_COMM_WORLD);
//        }
//        for (int i = 1; i < mpi_size; ++i) {
//            std::pair<int, int> output;
//            MPI_Recv(&output, 2, MPI_INT, i, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
//            //std::cout << i << "-th rank returned " << output << std::endl;
//            n += output.first;
//            nc += output.second;
//            std::cout << i << "-th rank pi=" << 4.0 * (double)output.second / output.first << std::endl;
//        }
//        std::cout << 4 * (double)nc / n << std::endl;
//    }
//    else {
//        int input;
//        MPI_Recv(&input, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
//        std::pair<int, int> output = iter(input);
//        MPI_Send(&output, 2, MPI_INT, 0, 0, MPI_COMM_WORLD);
//        std::cout << rank << " FIN" << std::endl;
//    }
//    MPI_Finalize();
//}

//--------find product of matrices, 2 processors

int main(int argc, char* argv[])
{
    MPI_Init(nullptr, nullptr);
    
    int rank, mpi_size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);
    int n = 4;
    int* a = nullptr;
    int* b = new int [n*n];

    if (rank == 0) {
        a = new int[n * n];
        for (int i = 0; i < n*n; i++) {
            a[i] = i + 1; 
            b[i] = i + 2;
        }
    }
    MPI_Bcast(b, n * n, MPI_INT, 0, MPI_COMM_WORLD);
    int* ha = new int[n * n / 2];
    MPI_Scatter(a, n * n / 2, MPI_INT, ha, n * n / 2, MPI_INT, 0, MPI_COMM_WORLD);
    int* hc = new int[n * n / 2];
    for (int i = 0; i < n / 2;i++) {
        for (int j = 0; j < n; j++) {
            hc[n * i + j] = 0;
            for (int k = 0; k < n; k++) {
                hc[n * i + j] += ha[n * i + k] * b[k * n + j];
            }
        }
    }
    int* c = new int[n * n];
    MPI_Gather(hc,n*n/2,MPI_INT, c, n*n/2, MPI_INT, 0,MPI_COMM_WORLD);
    if (rank == 0) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                std::cout << c[n * i + j] << " ";
            }
            std::cout << std::endl;
        }
        delete[]a;
    }
    delete[]b;
    delete[]ha;
    delete[]hc;
    delete[]c;
    MPI_Finalize();
}