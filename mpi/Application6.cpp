#include "mpi.h"
#include "timer.hpp"
#include <iostream>
#include <random>
#include <string>
#include <vector>
std::mt19937 gen(0);

double heavyfunc(double init)
{
    int n = 5'000;
    double r = 0;
    for (int i = 0; i < n; ++i)
    {
        for (int j = 0; j < n; ++j)
        {
            r += cos((double)i + init) * sin(init * (double)j) * pow((double)i / 100, 1.23123);
        }
    }
    return r;
}

std::pair<int, int> iter(int seed)
{
    std::mt19937 gen(seed);
    std::uniform_real_distribution<> dx(-1, 1);
    int n = 0;
    int nc = 0;
    while (n < 1e6 / 3)
    {
        double x = dx(gen);
        double y = dx(gen);
        if (x * x + y * y < 1)
        {
            nc++;
        }
        n++;
    }
    return {n, nc};
}

//--------find pi
void compute_pi()
{

    int rank, mpi_size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);
    int n = 0;
    int nc = 0;
    if (rank == 0)
    {
        for (int i = 1; i < mpi_size; ++i)
        {
            int input = i;
            MPI_Send(&input, 1, MPI_INT, i, 0, MPI_COMM_WORLD);
        }
        for (int i = 1; i < mpi_size; ++i)
        {
            std::pair<int, int> output;
            MPI_Recv(&output, 2, MPI_INT, i, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            // std::cout << i << "-th rank returned " << output << std::endl;
            n += output.first;
            nc += output.second;
            std::cout << i << "-th rank pi=" << 4.0 * (double)output.second / output.first << std::endl;
        }
        std::cout << 4 * (double)nc / n << std::endl;
    }
    else
    {
        int input;
        MPI_Recv(&input, 1, MPI_INT, 0, 0, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        std::pair<int, int> output = iter(input);
        MPI_Send(&output, 2, MPI_INT, 0, 0, MPI_COMM_WORLD);
        std::cout << rank << " FIN" << std::endl;
    }
}

//--------find product of matrices, 2 processors
void matrix_mult_n2()
{
    int rank, mpi_size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);
    int n = 4;
    int* a = nullptr;
    int* b = new int[n * n];
    int* c = nullptr;
    int* ha = new int[n * n / 2];
    int* hc = new int[n * n / 2];

    if (rank == 0)
    {
        c = new int[n * n];
        a = new int[n * n];
        for (int i = 0; i < n * n; i++)
        {
            a[i] = i + 1;
            b[i] = i + 2;
        }
    }

    MPI_Bcast(b, n * n, MPI_INT, 0, MPI_COMM_WORLD);
    MPI_Scatter(a, n * n / 2, MPI_INT, ha, n * n / 2, MPI_INT, 0, MPI_COMM_WORLD);
    for (int i = 0; i < n / 2; i++)
    {
        for (int j = 0; j < n; j++)
        {
            hc[n * i + j] = 0;
            for (int k = 0; k < n; k++)
            {
                hc[n * i + j] += ha[n * i + k] * b[k * n + j];
            }
        }
    }
    MPI_Gather(hc, n * n / 2, MPI_INT, c, n * n / 2, MPI_INT, 0, MPI_COMM_WORLD);

    if (rank == 0)
    {
        for (int i = 0; i < n; i++)
        {
            for (int j = 0; j < n; j++)
            {
                std::cout << c[n * i + j] << " ";
            }
            std::cout << std::endl;
        }
        delete[] a;
        delete[] c;
    }
    delete[] b;
    delete[] ha;
    delete[] hc;
}

//--------find product of matrices
void matrix_mult(int argc, char* argv[])
{
    int N = std::stoi(argv[1]);
    int M = std::stoi(argv[2]);

    if (argc != 3 || N <= 0 || M <= 0)
    {
        throw std::runtime_error("incorrect input");
    }
    int rank, mpi_size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);

    int* a = nullptr;
    int* c = nullptr;
    int* b = new int[M * N];

    int nrows = std::ceil(double(N) / mpi_size);
    int* ha = new int[nrows * M]();
    int* hc = new int[nrows * N]();
    if (rank == 0)
    {
        a = new int[(nrows * mpi_size) * M]();
        c = new int[(nrows * mpi_size) * N]();

        for (int i = 0; i < N * M; i++)
        {
            a[i] = i + 1;
            b[i] = i - 1;
        }
    }
    MPI_Bcast(b, M * N, MPI_INT, 0, MPI_COMM_WORLD);

    MPI_Scatter(a, nrows * M, MPI_INT, ha, nrows * M, MPI_INT, 0, MPI_COMM_WORLD);

    for (int i = 0; i < nrows; i++)
    {
        for (int j = 0; j < N; j++)
        {
            for (int k = 0; k < M; k++)
            {
                hc[i * N + j] += ha[i * M + k] * b[k * N + j];
            }
        }
    }

    MPI_Gather(hc, nrows * N, MPI_INT, c, nrows * N, MPI_INT, 0, MPI_COMM_WORLD);

    if (rank == 0)
    {
        if (std::max(N, M) <= 10)
        {
            std::cout << std::endl << " ====Matrix A===" << std::endl;
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    std::cout << a[M * i + j] << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl << " ====Matrix B===" << std::endl;
            for (int i = 0; i < M; i++)
            {
                for (int j = 0; j < N; j++)
                {
                    std::cout << b[N * i + j] << " ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl << " ====Matrix C===" << std::endl;
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < N; j++)
                {
                    std::cout << c[N * i + j] << " ";
                }
                std::cout << std::endl;
            }
        }
        delete[] a;
        delete[] c;
    }
    delete[] b;
    delete[] ha;
    delete[] hc;
}

//--------solution of SLAE
void solution_slae(int argc, char* argv[], int& rank, int& mpi_size)
{
    int N = std::stoi(argv[1]);
    if (N <= 0 || N > 10'000)
    {
        throw std::runtime_error("incorrect input");
    }

    int nrows = std::ceil(double(N) / mpi_size);

    std::vector<double> A(N * nrows * mpi_size, 0.0);
    std::vector<double> b(nrows * mpi_size, 0.0);
    std::vector<double> x(nrows * mpi_size, 0.0);

    // fill matrix

    if (rank == 0)
    {
        // N = 3;
        // A[0] = 2; A[1] = 1; A[2] = -1;
        // A[3] = -3; A[4] = -1; A[5] = 2;
        // A[6] = -2; A[7] = 1; A[8] = 2;
        // b[0] = 8; b[1] = -11; b[2] = -3;

        // N = 4;
        // A = {2, 3, 1, 4,
        //    3, 4, 2, 4,
        //    2, 1, 1, 2,
        //    4, 4, 3, 2};
        // b = {2, 1, 3, 1};

        std::uniform_real_distribution<double> distr(1, 100);
        for (int i = 0; i < N; i++)
        {
            b[i] = distr(gen);
            for (int j = 0; j < N; j++)
            {
                A[i * N + j] = distr(gen);
            }
        }
    }

    std::vector<double> parta(N * nrows);
    std::vector<double> partb(nrows);
    MPI_Scatter(A.data(), N * nrows, MPI_DOUBLE, parta.data(), N * nrows, MPI_DOUBLE, 0, MPI_COMM_WORLD);
    MPI_Scatter(b.data(), nrows, MPI_DOUBLE, partb.data(), nrows, MPI_DOUBLE, 0, MPI_COMM_WORLD);

    // forward elimination
    for (int i = 0; i < N; i++)
    {
        int cur_rank = i / nrows;

        if (rank < cur_rank)
        {
            continue;
        }

        std::vector<double> subtrrow(N);
        double subtrb;

        if (rank == cur_rank)
        {
            int irow = i - nrows * rank;
            double diag = parta[N * irow + i];
            partb[irow] /= diag;
            for (int j = irow * N; j < irow * N + N; j++)
            {
                parta[j] /= diag;
            }

            for (int j = irow * N; j < irow * N + N; ++j)
            {
                subtrrow[j - irow * N] = parta[j];
            }
            subtrb = partb[irow];
            for (int k = irow + 1; k < nrows; k++)
            {
                double coef = parta[k * N + i];
                partb[k] -= coef * subtrb;
                for (int j = 0; j < N; j++)
                {
                    parta[k * N + j] -= subtrrow[j] * coef;
                }
            }

            for (int rank2 = cur_rank + 1; rank2 < mpi_size; ++rank2)
            {
                MPI_Send(subtrrow.data(), N, MPI_DOUBLE, rank2, 1, MPI_COMM_WORLD);
                MPI_Send(&subtrb, 1, MPI_DOUBLE, rank2, 1, MPI_COMM_WORLD);
            }
        }

        else if (rank > cur_rank)
        {
            MPI_Recv(subtrrow.data(), N, MPI_DOUBLE, cur_rank, 1, MPI_COMM_WORLD, MPI_STATUSES_IGNORE);
            MPI_Recv(&subtrb, 1, MPI_DOUBLE, cur_rank, 1, MPI_COMM_WORLD, MPI_STATUSES_IGNORE);

            for (int k = 0; k < nrows; k++)
            {
                double coef = parta[k * N + i];
                partb[k] -= coef * subtrb;
                for (int j = 0; j < N; j++)
                {
                    parta[k * N + j] -= subtrrow[j] * coef;
                }
            }
        }
    }

    // back substitution
    for (int k = N - 1; k >= 0; k--)
    {
        int cur_rank = k / nrows;
        if (rank < cur_rank)
        {
            continue;
        }

        if (rank > cur_rank)
        {
            MPI_Send(x.data() + rank * nrows, nrows, MPI_DOUBLE, cur_rank, 1, MPI_COMM_WORLD);
        }

        else if (rank == cur_rank)
        {
            for (int irank = cur_rank + 1; irank < mpi_size; irank++)
            {
                MPI_Recv(x.data() + irank * nrows, nrows, MPI_DOUBLE, irank, 1, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
            }

            int irow = k - nrows * cur_rank;
            x[k] = partb[irow];
            for (int j = k + 1; j < N; j++)
            {
                x[k] -= parta[irow * N + j] * x[j];
            }
        }
    }

    // print
    if (rank == 0 && N < 15)
    {
        for (int i = 0; i < N; i++)
        {
            std::cout << "x_" << i << " = " << x[i] << std::endl;
        }
    }
}

//--------solution of SLAE
void solution_slae_strings(int argc, char* argv[], int& rank, int& mpi_size)
{
    int N = std::stoi(argv[1]);
    if (N <= 0 || N > 10'000)
    {
        throw std::runtime_error("incorrect input");
    }

    int nrows = std::ceil(double(N) / mpi_size);

    std::vector<double> A(N * nrows * mpi_size, 0.0);
    std::vector<double> b(nrows * mpi_size, 0.0);
    std::vector<double> x(nrows * mpi_size, 0.0);

    std::vector<double> local_A(N * nrows, 0.0);
    std::vector<double> local_b(nrows, 0.0);

    // fill matrix
    if (rank == 0)
    {
        // A = { 2, 3, 1, 4, 5,     //
        //	  3, 4, 2, 4, 4,   //
        //	  2, 1, 1, 2, 3,   //
        //	  4, 4, 3, 2, 4,
        //	  1, 2, 4, 1, 5 };  //
        // b = { 2, 1, 3, 1, 2 };
        std::uniform_real_distribution<double> distr(1, 100);
        for (int i = 0; i < N; i++)
        {
            b[i] = distr(gen);
            for (int j = 0; j < N; j++)
            {
                A[i * N + j] = distr(gen);
            }
        }

        MPI_Datatype row_type, partb_type;

        MPI_Type_vector(nrows, N, mpi_size * N, MPI_DOUBLE, &row_type);
        MPI_Type_commit(&row_type);
        MPI_Type_vector(nrows, 1, mpi_size, MPI_DOUBLE, &partb_type);
        MPI_Type_commit(&partb_type);

        MPI_Sendrecv(A.data(), 1, row_type, 0, 1, local_A.data(), nrows * N, MPI_DOUBLE, 0, 1, MPI_COMM_WORLD,
                     MPI_STATUS_IGNORE);
        MPI_Sendrecv(b.data(), 1, partb_type, 0, 1, local_b.data(), nrows, MPI_DOUBLE, 0, 1, MPI_COMM_WORLD,
                     MPI_STATUS_IGNORE);
        for (int i = 1; i < mpi_size; i++)
        {
            MPI_Send(A.data() + N * i, 1, row_type, i, 2, MPI_COMM_WORLD);
            MPI_Send(b.data() + i, 1, partb_type, i, 0, MPI_COMM_WORLD);
        }
    }
    else
    {
        MPI_Recv(local_A.data(), nrows * N, MPI_DOUBLE, 0, 2, MPI_COMM_WORLD, MPI_STATUSES_IGNORE);
        MPI_Recv(local_b.data(), nrows, MPI_DOUBLE, 0, 0, MPI_COMM_WORLD, MPI_STATUSES_IGNORE);
    }

    // forward elimination
    for (int i = 0; i < N; i++)
    {
        int cur_rank = i % mpi_size;

        std::vector<double> send_A(N, 0.0);
        double send_b = 0;

        if (rank == cur_rank)
        {
            int irow = i / mpi_size;
            double diag = local_A[irow * N + i];

            local_b[irow] /= diag;
            send_b = local_b[irow];
            for (int k = irow * N; k < irow * N + N; k++)
            {
                local_A[k] /= diag;
                send_A[k - irow * N] = local_A[k];
            }

            for (int j = irow + 1; j < nrows; j++)
            {
                double coef = local_A[j * N + i];
                local_b[j] -= send_b * coef;
                for (int k = j * N; k < j * N + N; k++)
                {
                    local_A[k] -= send_A[k - j * N] * coef;
                }
            }
        }

        MPI_Bcast(send_A.data(), N, MPI_DOUBLE, cur_rank, MPI_COMM_WORLD);
        MPI_Bcast(&send_b, 1, MPI_DOUBLE, cur_rank, MPI_COMM_WORLD);

        if (rank != cur_rank)
        {
            for (int j = 0; j < nrows; j++)
            {
                if (mpi_size * j + rank < i)
                {
                    continue;
                }
                double coef = local_A[j * N + i];
                local_b[j] -= send_b * coef;
                for (int k = j * N; k < j * N + N; k++)
                {
                    local_A[k] -= send_A[k - j * N] * coef;
                }
            }
        }
    }

    // back substitution
    for (int i = N - 1; i >= 0; i--)
    {
        int cur_rank = i % mpi_size;

        if (rank == cur_rank)
        {
            int irow = i / mpi_size;
            x[i] = local_b[irow];

            for (int j = i + 1; j < N; j++)
            {
                int local_rank = j % mpi_size;
                if (local_rank != rank)
                {
                    MPI_Recv(x.data() + j, 1, MPI_DOUBLE, local_rank, 0, MPI_COMM_WORLD, MPI_STATUSES_IGNORE);
                }
                x[i] -= local_A[irow * N + j] * x[j];
            }
        }

        else
        {
            for (int j = i + 1; j < N; j++)
            {
                if (rank == j % mpi_size)
                    MPI_Send(x.data() + j, 1, MPI_DOUBLE, cur_rank, 0, MPI_COMM_WORLD);
            }
        }
    }

    // print result
    if (rank == 0 && N < 12)
    {
        for (int i = 0; i < N; i++)
        {
            std::cout << "x[" << i << "]= " << x[i] << std::endl;
        }
    }
}

int main(int argc, char* argv[])
{
    MPI_Init(&argc, &argv);
    // compute_pi();
    // matrix_mult_n2();
    // matrix_mult(argc, argv);
    // solution_slae(argc, argv);
    int rank, mpi_size;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &mpi_size);
    Timer t;
    int time = 0;
    for (int i = 0; i < 20; i++)
    {
        t.start();
        solution_slae(argc, argv, rank, mpi_size);
        if (rank == mpi_size - 1) time += t.end();
    }
    //for (int i = 0; i < 20; i++)
    //{
    //    t.start();
    //    solution_slae_strings(argc, argv, rank, mpi_size);
    //    if (rank == mpi_size - 1) time += t.end();
    //}

    std::cout << time / 20 << std::endl;
    MPI_Finalize();
}
