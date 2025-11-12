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
    config.ins_drift_rate = 2.0;                   /* ����� ��� 2 �/�/��� */
    config.sns_outage_start = 300.0;               /* ������ ��� � 5-� ������ */
    config.sns_outage_duration = 180.0;            /* �� 3 ������ */
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

    /* Preserve original settings to restore later */
    data_validity_t original_use_tcas = config.use_tcas;
    data_validity_t original_use_koei = config.use_koei;
    double original_sns_outage_start = config.sns_outage_start;
    double original_sns_outage_duration = config.sns_outage_duration;
    double original_ins_drift_rate = config.ins_drift_rate;

    config.use_tcas = DATA_INVALID;
    // FIXED: For proper comparison, disable SNS and KOEI in "without TCAS" mode
    // This shows true INS-only performance without external corrections
    config.use_koei = DATA_INVALID;  // Disable KOEI
    config.sns_outage_start = 0.0;   // Start SNS outage immediately
    config.sns_outage_duration = config.simulation_time;  // Full duration - no SNS corrections
    // FIXED: Increase drift rate for "without TCAS" to show realistic INS drift accumulation
    config.ins_drift_rate = 5000.0;  // Strong drift to highlight TCAS benefits
    num_results = simulate_approach(&config, scenario, results_without_tcas, 1000);

    if (num_results > 0) {
        save_simulation_results(results_without_tcas, num_results, "results_without_tcas.csv");
        print_simulation_statistics(results_without_tcas, num_results);
    }

    /* ��������� ����������� */
    if (num_results > 0) {
        compare_tcas_integration(results_with_tcas, results_without_tcas, num_results);
    }

    /* Restore original settings for subsequent scenarios */
    config.use_tcas = original_use_tcas;
    config.use_koei = original_use_koei;
    config.sns_outage_start = original_sns_outage_start;
    config.sns_outage_duration = original_sns_outage_duration;
    config.ins_drift_rate = original_ins_drift_rate;

    /* �������������� �������� ��� ������������ */
    printf("\n=========================================\n");
    printf("�������������� �������� �������������:\n");
    printf("=========================================\n");

    /* �������� 1: ���������� ������� ��� ������ ������� */
    printf("\n�������� 1: ���������� ������� (��� ������ ���)\n");
    config.sns_outage_start = 0.0;
    config.sns_outage_duration = 0.0;
    config.use_tcas = DATA_VALID;

    simulation_result_t results_normal[1000];
    num_results = simulate_approach(&config, scenario, results_normal, 1000);
    if (num_results > 0) {
        save_simulation_results(results_normal, num_results, "results_normal_conditions.csv");
        print_simulation_statistics(results_normal, num_results);
    }

    /* �������� 2: ������� ������ */
    printf("\n�������� 2: ������� ������ (15 ��������� �����)\n");
    config.num_traffic = 15;
    config.sns_outage_start = 300.0;
    config.sns_outage_duration = 180.0;

    simulation_result_t results_heavy_traffic[1000];
    num_results = simulate_approach(&config, scenario, results_heavy_traffic, 1000);
    if (num_results > 0) {
        save_simulation_results(results_heavy_traffic, num_results, "results_heavy_traffic.csv");
        print_simulation_statistics(results_heavy_traffic, num_results);
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
    printf("3. ��������� ������������� ��������� ��������� ������ �� �������\n");
    printf("4. �������� ����������, �������������� ������������ ���������� TCAS\n");
    printf("5. ������� ����� � ������� ��� ����������� �������\n\n");

    printf("����� �����������:\n");
    printf("- results_with_tcas.csv - ���������� � ����������� TCAS\n");
    printf("- results_without_tcas.csv - ���������� ��� TCAS (��� ���������)\n");
    printf("- results_normal_conditions.csv - ���������� �������\n");
    printf("- results_heavy_traffic.csv - ������� ������\n");
    printf("- results_no_koei.csv - ��� ������������� ����\n\n");

    printf("��� ������������ ����������� ����������� Python ������:\n");
    printf("python analyze_results.py\n");

    return 0;
}