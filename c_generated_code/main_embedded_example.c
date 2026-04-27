#include <stdio.h>
#include <stdlib.h>

// Główne nagłówki interfejsu Acadosa
#include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"

// Twój wygenerowany nagłówek z modelem Segwaya
#include "acados_solver_segway_nonlinear_mpc.h"

// Przydatne makra wygenerowane przez Acados
#define NX SEGWAY_NONLINEAR_MPC_NX // Liczba stanów (6)
#define NU SEGWAY_NONLINEAR_MPC_NU // Liczba wejść (2)

int main(void) {
  // Wymuszenie natychmiastowego wypisywania na ekran (brak buforowania)
  setvbuf(stdout, NULL, _IONBF, 0);

  /* ========================================================================
   * 1. INICJALIZACJA SOLVERA (Wywoływane tylko raz na starcie systemu)
   * ======================================================================== */

  // Utworzenie kapsuły solvera (struktura trzymająca wskaźniki i pamięć)
  segway_nonlinear_mpc_solver_capsule *capsule =
      segway_nonlinear_mpc_acados_create_capsule();

  // Alokacja pamięci i konfiguracja macierzy problemu MPC
  int status = segway_nonlinear_mpc_acados_create(capsule);

  if (status != 0) {
    printf("Fatal Error: Nie udalo sie zainicjowac solvera MPC (status: %d)\n",
           status);
    return 1;
  }
  printf("Solver MPC zainicjowany pomyslnie. Gotowy do pracy!\n");

  /* ========================================================================
   * 2. GŁÓWNA PĘTLA STEROWANIA (Wywoływana cyklicznie np. co 20ms / 50Hz)
   * ======================================================================== */

  // Tablice na stany i sterowania
  double current_x[NX];
  double u_opt[NU];

  double elapsed_time = 0.0;
  ocp_nlp_solver *nlp_solver =
      segway_nonlinear_mpc_acados_get_nlp_solver(capsule);

  // Na potrzeby testu na PC ograniczamy pętlę do np. 10 iteracji
  int iter = 0;
  while (iter++ < 10) {
    // --- KROK A: Pobranie stanu robota ---
    // Tutaj wstawiasz kod odczytujący dane z IMU (pitch, gyro) i enkoderów kół
    // (położenie, prędkość) Stan: [X, X_dot, Phi, Phi_dot, Psi, Psi_dot]
    current_x[0] = 0.0;  // X (pozycja)
    current_x[1] = 0.0;  // X_dot (predkosc liniowa)
    current_x[2] = 0.15; // Phi (np. robot pochylony o ok. 8.5 stopnia)
    current_x[3] = 0.0;  // Phi_dot (predkosc opadania)
    current_x[4] = 0.0;  // Psi (kat obrotu - yaw)
    current_x[5] = 0.0;  // Psi_dot (predkosc obrotu)

    // --- KROK B: Wstawienie obecnego stanu jako warunek początkowy do MPC ---
    // Wymuszamy, by optymalizator zaczął przewidywanie dokładnie od obecnego
    // stanu robota
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims,
                                  capsule->nlp_in, capsule->nlp_out, 0, "lbx",
                                  current_x);
    ocp_nlp_constraints_model_set(capsule->nlp_config, capsule->nlp_dims,
                                  capsule->nlp_in, capsule->nlp_out, 0, "ubx",
                                  current_x);

    // --- KROK C: Obliczenie optymalnego sterowania ---
    status = segway_nonlinear_mpc_acados_solve(capsule);

    if (status == 0) // ACADOS_SUCCESS
    {
      // --- KROK D: Wyciągnięcie odpowiedzi (tylko dla pierwszego kroku w
      // horyzoncie) ---
      ocp_nlp_out_get(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_out,
                      0, "u", u_opt);

      ocp_nlp_get(nlp_solver, "time_tot", &elapsed_time);
      printf("Iter: %d | Sterowanie: u1=%7.3f, u2=%7.3f | Czas: %6.3f ms\n",
             iter, u_opt[0], u_opt[1], elapsed_time * 1000.0);

      // Teraz masz w u_opt optymalne przyśpieszenia kół!
      // set_motor_acceleration(LEFT_MOTOR,  u_opt[0]);
      // set_motor_acceleration(RIGHT_MOTOR, u_opt[1]);
    } else {
      // Tryb awaryjny - wyłącz silniki gdy model rozbiegnie
      // stop_motors();
      printf("MPC FAIL! Status: %d\n", status);
    }

    // np. vTaskDelay(pdMS_TO_TICKS(20)); // Czekaj na kolejny krok w systemie
    // RTOS
  }

  printf("Koniec symulacji.\n");

  /* ========================================================================
   * 3. CZYSZCZENIE PAMIĘCI (zwykle mikrokontroler tu nie dociera)
   * ======================================================================== */
  segway_nonlinear_mpc_acados_free(capsule);
  segway_nonlinear_mpc_acados_free_capsule(capsule);

  return 0;
}
