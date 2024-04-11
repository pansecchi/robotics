#include <iostream>
#include <cmath>
// Definizione della costante pi greco
const double PI = 3.14159265358979323846;

// Funzione per convertire le coordinate da ECEF a ENU


void ECEF_to_ENU(const double ecef[3], const double ref_ecef[3], double enu[3]) {
    // Calcola la latitudine e la longitudine del punto di riferimento
    double latit = std::atan2(ref_ecef[2], std::sqrt(ref_ecef[0] * ref_ecef[0] + ref_ecef[1] * ref_ecef[1]));
    double longit = std::atan2(ref_ecef[1], ref_ecef[0]);
    // Calcola la matrice di rotazione da ECEF a ENU
    double rotation_matrix[3][3] = {
        {-std::sin(longit), std::cos(longit), 0},
        {-std::sin(latit) * std::cos(longit), -std::sin(latit) * std::sin(longit), std::cos(latit)},
        {std::cos(latit) * std::cos(longit), std::cos(latit) * std::sin(longit), std::sin(latit)}
    };
    // Calcola il vettore di traslazione da ECEF a ENU
    double translation_vector[3] = {-ref_ecef[0], -ref_ecef[1], -ref_ecef[2]};


 
    // Moltiplica le coordinate ECEF del punto di interesse per la matrice di rotazione e aggiungi il vettore di traslazione
    for (int i = 0; i < 3; ++i) {
        enu[i] = 0;
        for (int j = 0; j < 3; ++j) {
            enu[i] += rotation_matrix[i][j] * (ecef[j] + translation_vector[j]);
        }
    }
}



int main() {
    // Coordinate ECEF del punto di interesse (in metri)
    double ecef[3] = {6378137.0, 0.0, 0.0};
    // Coordinate ECEF del punto di riferimento (in metri)
    double ref_ecef[3] = {6378137.0, 0.0, 0.0};
    // Variabile per le coordinate ENU (East-North-Up)
    double enu[3];
    // Converti le coordinate da ECEF a ENU
    ECEF_to_ENU(ecef, ref_ecef, enu);
    // Stampa le coordinate ENU
    std::cout << "East: " << enu[0] << " meters" << std::endl;
    std::cout << "North: " << enu[1] << " meters" << std::endl;
    std::cout << "Up: " << enu[2] << " meters" << std::endl;
    return 0;
}