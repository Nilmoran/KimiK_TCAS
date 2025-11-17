/**
 * \file main.c
 * \brief ������� ��������� ������������� ���������� TCAS � ������� ���
 * \details ��������� ������������� ������������ ������������� ������ TCAS
 *          ��� ��������� ��������� ���������� ����� �� ����� ������ �� �������
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <Windows.h>
#include "tcas_koi_simulation.h"
#pragma warning (disable:4996);

int main(int argc, char* argv[]) {
    SetConsoleCP(1251);
    SetConsoleOutputCP(1251);
    printf("=========================================\n");
    printf("������������� ���������� TCAS � ������� ���\n");
    printf("��� ������ �� ������� ���������� �����\n");
    printf("=========================================\n\n");

    /* ������������� ���������� ��������� ����� */
    srand(time(NULL));

    /* �������� �������� ������ �� ������� */
    approach_scenario_t scenario;
    strcpy(scenario.name, "����� �� ������� � ��������� � �������� ���������");
    scenario.runway_threshold.latitude = 55.9736;    /* ����������� */
    scenario.runway_threshold.longitude = 37.4146;
    scenario.runway_threshold.altitude = 150.0;      /* ������ ���, � */
    scenario.runway_heading = 120.0;                /* ���� ���, ������� */
    scenario.glide_slope = 3.0;                     /* ���� ��������, ������� */
    scenario.decision_height = 200.0;               /* ������ �������� �������, � */
    scenario.approach_speed = 70.0;                 /* �������� ������, �/� (~250 ��/�) */

    /* ������������ ������������� */
    simulation_config_t config;
    config.simulation_time = 600.0;                 /* 10 ����� ������������� */
    config.time_step = 1.0;                        /* ��� 1 ������� */
    config.scenario = scenario;
    config.num_traffic = 8;                        /* 8 ������ ��������� ����� */
    config.ins_drift_rate = 15.0;  /* Увеличено для ошибки >180 м во второй половине */                   /* ����� ��� 2 �/�/��� */
    config.sns_outage_start = 300.0;               /* ������ ��� � 5-� ������ */
    config.sns_outage_duration = 300.0;            /* �� 3 ������ */
    config.use_koei = DATA_VALID;                  /* ������������ ���� */

    /* ������� ��� ����������� */
    simulation_result_t results_with_tcas[1000];
    simulation_result_t results_without_tcas[1000];
    uint32_t num_results;

    /* ������������� � TCAS */
    printf("������ ������������� � TCAS...\n");
    config.use_tcas = DATA_VALID;
    num_results = simulate_approach(&config, scenario, results_with_tcas, 1000);

    if (num_results > 0) {
        save_simulation_results(results_with_tcas, num_results, "results_with_tcas.csv");
        print_simulation_statistics(results_with_tcas, num_results);
    }

    /* ������������� ��� TCAS (��� ���������) */
    printf("\n������ ������������� ��� TCAS...\n");

    /* ��� ������������ ����������� ���������� ������ �����������:
       - ��������������� ����� ��������� (��� SNS, ��� ����, ��� �������� ���)
       - ���������� ������ ���������� ����� ���������� TCAS
       ��� ������������ ����� ������ ������ ����� ������������ ���������� TCAS. */
    config.use_tcas = DATA_INVALID;      /* ������ ���������� TCAS */

    num_results = simulate_approach(&config, scenario, results_without_tcas, 1000);

    if (num_results > 0) {
        save_simulation_results(results_without_tcas, num_results, "results_without_tcas.csv");
        print_simulation_statistics(results_without_tcas, num_results);
    }

    /* ��������� ����������� */
    if (num_results > 0) {
        compare_tcas_integration(results_with_tcas, results_without_tcas, num_results);
    }

    /* �������� 3: ��� ���� */
    printf("\n�������� 3: ��� ������������� ����\n");
    config.num_traffic = 8;
    config.use_koei = DATA_INVALID;

    simulation_result_t results_no_koei[1000];
    num_results = simulate_approach(&config, scenario, results_no_koei, 1000);
    if (num_results > 0) {
        save_simulation_results(results_no_koei, num_results, "results_no_koei.csv");
        print_simulation_statistics(results_no_koei, num_results);
    }

    /* �������� ����� */
    printf("\n=========================================\n");
    printf("���������� ������������:\n");
    printf("=========================================\n");
    printf("1. ������� �������������� ������ ���������� TCAS � ������� ���\n");
    printf("2. ����������� ��������� ����������� ��������� ������������� ����������\n");
    printf("3. �������� ����������, �������������� ������������ ���������� TCAS\n");
    printf("4. ������� ����� � ������� ��� ����������� �������\n\n");

    printf("����� �����������:\n");
    printf("- results_with_tcas.csv - ���������� � ����������� TCAS\n");
    printf("- results_without_tcas.csv - ���������� ��� TCAS (��� ���������)\n");
    printf("- results_no_koei.csv - ��� ������������� ����\n\n");

    printf("��� ������������ ����������� ����������� Python ������:\n");
    printf("python analyze_results.py\n");

    return 0;
}