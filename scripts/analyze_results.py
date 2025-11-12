#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Скрипт для анализа и визуализации результатов моделирования
интеграции TCAS в систему КОИ
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
from matplotlib import font_manager as fm
import warnings
warnings.filterwarnings('ignore')

# Настройка русского языка в matplotlib
fm.fontManager.__init__()
cjk_list = ['CJK', 'Han', 'CN', 'TW', 'JP']
cjk_fonts = [f.name for f in fm.fontManager.ttflist if any(s.lower() in f.name.lower() for s in cjk_list)]
plt.rcParams['font.family'] = ['DejaVu Sans'] + cjk_fonts
plt.rcParams['axes.unicode_minus'] = False

# Настройка стиля графиков
plt.style.use('seaborn-v0_8-whitegrid')
sns.set_palette("husl")

def load_results(filename):
    """Загрузка результатов моделирования из CSV файла"""
    try:
        df = pd.read_csv(filename)
        return df
    except FileNotFoundError:
        print(f"Файл {filename} не найден")
        return None
    except Exception as e:
        print(f"Ошибка при загрузке файла {filename}: {e}")
        return None

def plot_position_errors(df_list, labels, title):
    """Построение графика ошибок позиционирования"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    for df, label in zip(df_list, labels):
        if df is not None:
            ax1.plot(df['timestamp'], df['error_2d'], label=label, linewidth=2, alpha=0.8)
            ax2.plot(df['timestamp'], df['error_3d'], label=label, linewidth=2, alpha=0.8)
    
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('2D ошибка, м')
    ax1.set_title(f'{title} - 2D ошибка позиционирования')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.set_xlabel('Время, с')
    ax2.set_ylabel('3D ошибка, м')
    ax2.set_title(f'{title} - 3D ошибка позиционирования')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_velocity_errors(df_list, labels, title):
    """Построение графика ошибок скорости"""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    for df, label in zip(df_list, labels):
        if df is not None:
            ax.plot(df['timestamp'], df['velocity_error'], label=label, linewidth=2, alpha=0.8)
    
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Ошибка скорости, м/с')
    ax.set_title(f'{title} - Ошибка определения скорости')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    return fig

def plot_dop_comparison(df_list, labels, title):
    """Сравнение геометрических факторов точности"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    for df, label in zip(df_list, labels):
        if df is not None:
            ax1.plot(df['timestamp'], df['hdop'], label=label, linewidth=2, alpha=0.8)
            ax2.plot(df['timestamp'], df['vdop'], label=label, linewidth=2, alpha=0.8)
    
    ax1.set_xlabel('Время, с')
    ax1.set_ylabel('HDOP')
    ax1.set_title(f'{title} - Горизонтальный геометрический фактор')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.set_xlabel('Время, с')
    ax2.set_ylabel('VDOP')
    ax2.set_title(f'{title} - Вертикальный геометрический фактор')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_tcas_availability(df, title):
    """Построение графика доступности TCAS"""
    fig, ax = plt.subplots(figsize=(12, 6))
    
    # Создание массива цветов на основе доступности TCAS
    colors = ['red' if x == 0 else 'green' for x in df['tcas_available']]
    
    ax.scatter(df['timestamp'], df['tcas_targets'], 
              c=colors, alpha=0.6, s=50)
    
    ax.set_xlabel('Время, с')
    ax.set_ylabel('Количество целей TCAS')
    ax.set_title(f'{title} - Доступность TCAS')
    ax.grid(True, alpha=0.3)
    
    # Легенда
    from matplotlib.patches import Patch
    legend_elements = [Patch(facecolor='green', label='TCAS доступен'),
                      Patch(facecolor='red', label='TCAS недоступен')]
    ax.legend(handles=legend_elements)
    
    return fig

def plot_trajectory_3d(df_list, labels, title):
    """Построение 3D траектории полета"""
    fig = plt.figure(figsize=(14, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    colors = ['blue', 'red', 'green', 'orange', 'purple']
    
    for i, (df, label) in enumerate(zip(df_list, labels)):
        if df is not None:
            color = colors[i % len(colors)]
            ax.plot(df['est_lon'], df['est_lat'], df['est_alt'], 
                   label=label, color=color, linewidth=2, alpha=0.8)
    
    ax.set_xlabel('Долгота')
    ax.set_ylabel('Широта')
    ax.set_zlabel('Высота, м')
    ax.set_title(f'{title} - 3D траектория полета')
    ax.legend()
    
    return fig

def calculate_statistics(df):
    """Вычисление статистики по результатам моделирования"""
    stats = {
        'mean_2d_error': df['error_2d'].mean(),
        'std_2d_error': df['error_2d'].std(),
        'max_2d_error': df['error_2d'].max(),
        'mean_3d_error': df['error_3d'].mean(),
        'std_3d_error': df['error_3d'].std(),
        'max_3d_error': df['error_3d'].max(),
        'mean_velocity_error': df['velocity_error'].mean(),
        'mean_hdop': df['hdop'].mean(),
        'mean_vdop': df['vdop'].mean(),
        'tcas_availability': (df['tcas_available'] == 1).mean() * 100,
        'sns_availability': (df['sns_available'] == 1).mean() * 100
    }
    return stats

def create_comparison_table(stats_list, labels):
    """Создание таблицы сравнения статистики"""
    df_stats = pd.DataFrame(stats_list, index=labels)
    
    # Округление значений
    df_stats = df_stats.round(2)
    
    print("\nТаблица сравнения статистики:")
    print("=" * 80)
    print(df_stats.to_string())
    print("=" * 80)
    
    return df_stats

def plot_error_distribution(df_list, labels, title):
    """Построение распределения ошибок"""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    for df, label in zip(df_list, labels):
        if df is not None:
            ax1.hist(df['error_2d'], bins=30, alpha=0.6, label=label, density=True)
            ax2.hist(df['error_3d'], bins=30, alpha=0.6, label=label, density=True)
    
    ax1.set_xlabel('2D ошибка, м')
    ax1.set_ylabel('Плотность вероятности')
    ax1.set_title(f'{title} - Распределение 2D ошибок')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    ax2.set_xlabel('3D ошибка, м')
    ax2.set_ylabel('Плотность вероятности')
    ax2.set_title(f'{title} - Распределение 3D ошибок')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def main():
    """Основная функция анализа результатов"""
    import os
    
    # Выводим информацию о текущей директории
    print("Текущая рабочая директория:", os.getcwd())
    print("Все файлы в директории:", os.listdir())
    
    # Загрузка данных
    print("Загрузка результатов моделирования...")
    files = {
        'С TCAS': 'results_with_tcas.csv',
        'Без TCAS': 'results_without_tcas.csv',
        'Нормальные условия': 'results_normal_conditions.csv',
        'Плотный трафик': 'results_heavy_traffic.csv',
        'Без КЭОИ': 'results_no_koei.csv'
    }
    
    data = {}
    for label, filename in files.items():
        print(f"\n=== Попытка загрузить: {label} ===")
        
        # Проверяем наличие файла
        if not os.path.exists(filename):
            print(f"[ERROR] Файл {filename} не найден в текущей директории")
            # Проверяем в директории скрипта
            script_dir = os.path.dirname(os.path.abspath(__file__))
            alt_path = os.path.join(script_dir, filename)
            print(f"Проверяем альтернативный путь: {alt_path}")
            if os.path.exists(alt_path):
                print(f"[OK] Файл найден по альтернативному пути")
                filename = alt_path
            else:
                print(f"[ERROR] Файл не найден и по альтернативному пути")
                continue
        
        df = load_results(filename)
        if df is not None:
            data[label] = df
            print(f"[OK] Загружены данные: {label} ({len(df)} точек)")
        else:
            print(f"[ERROR] Не удалось загрузить данные для: {label}")
    
    if not data:
        print("[ERROR] Нет данных для анализа!")
        return
    
    # Вычисление статистики
    print("\nВычисление статистики...")
    stats_list = []
    labels = []
    
    for label, df in data.items():
        stats = calculate_statistics(df)
        stats_list.append(stats)
        labels.append(label)
    
    # Создание таблицы сравнения
    df_comparison = create_comparison_table(stats_list, labels)
    
    # Сохранение статистики в CSV
    df_comparison.to_csv('simulation_statistics.csv')
    print("\nСтатистика сохранена в файл: simulation_statistics.csv")
    
    # Построение графиков
    print("\nПостроение графиков...")
    
    # 1. Сравнение основных сценариев
    if 'С TCAS' in data and 'Без TCAS' in data:
        fig1 = plot_position_errors([data['С TCAS'], data['Без TCAS']], 
                                   ['С TCAS', 'Без TCAS'], 
                                   'Сравнение интеграции TCAS')
        fig1.savefig('position_error_comparison.png', dpi=300, bbox_inches='tight')
        plt.close(fig1)
        
        fig2 = plot_velocity_errors([data['С TCAS'], data['Без TCAS']], 
                                   ['С TCAS', 'Без TCAS'], 
                                   'Сравнение интеграции TCAS')
        fig2.savefig('velocity_error_comparison.png', dpi=300, bbox_inches='tight')
        plt.close(fig2)
        
        fig3 = plot_error_distribution([data['С TCAS'], data['Без TCAS']], 
                                      ['С TCAS', 'Без TCAS'], 
                                      'Сравнение интеграции TCAS')
        fig3.savefig('error_distribution_comparison.png', dpi=300, bbox_inches='tight')
        plt.close(fig3)
    
    # 2. Анализ всех сценариев
    all_data = list(data.values())
    all_labels = list(data.keys())
    
    fig4 = plot_position_errors(all_data, all_labels, 'Все сценарии')
    fig4.savefig('all_scenarios_position_errors.png', dpi=300, bbox_inches='tight')
    plt.close(fig4)
    
    # 3. 3D траектории
    if len(all_data) >= 2:
        fig5 = plot_trajectory_3d(all_data[:3], all_labels[:3], 'Сравнение траекторий')
        fig5.savefig('trajectory_comparison_3d.png', dpi=300, bbox_inches='tight')
        plt.close(fig5)
    
    # 4. Доступность TCAS
    if 'С TCAS' in data:
        fig6 = plot_tcas_availability(data['С TCAS'], 'Доступность TCAS')
        fig6.savefig('tcas_availability.png', dpi=300, bbox_inches='tight')
        plt.close(fig6)
    
    # Создание итогового отчета
    print("\nСоздание итогового отчета...")
    
    with open('analysis_report.txt', 'w', encoding='utf-8') as f:
        f.write("АНАЛИЗ РЕЗУЛЬТАТОВ МОДЕЛИРОВАНИЯ ИНТЕГРАЦИИ TCAS В СИСТЕМУ КОИ\n")
        f.write("=" * 80 + "\n\n")
        
        f.write("1. ОБЩАЯ ХАРАКТЕРИСТИКА ИССЛЕДОВАНИЯ\n")
        f.write("-" * 40 + "\n")
        f.write(f"Количество сценариев: {len(data)}\n")
        f.write(f"Общее время моделирования: {max([df['timestamp'].max() for df in data.values()])} секунд\n")
        f.write(f"Количество измерений: {[len(df) for df in data.values()]}\n\n")
        
        f.write("2. СТАТИСТИЧЕСКИЕ ПОКАЗАТЕЛИ\n")
        f.write("-" * 40 + "\n")
        f.write(df_comparison.to_string())
        f.write("\n\n")
        
        f.write("3. КЛЮЧЕВЫЕ ВЫВОДЫ\n")
        f.write("-" * 40 + "\n")
        
        if 'С TCAS' in data and 'Без TCAS' in data:
            improvement_2d = ((df_comparison.loc['Без TCAS', 'mean_2d_error'] - 
                              df_comparison.loc['С TCAS', 'mean_2d_error']) / 
                             df_comparison.loc['Без TCAS', 'mean_2d_error']) * 100
            improvement_3d = ((df_comparison.loc['Без TCAS', 'mean_3d_error'] - 
                              df_comparison.loc['С TCAS', 'mean_3d_error']) / 
                             df_comparison.loc['Без TCAS', 'mean_3d_error']) * 100
            
            f.write(f"• Улучшение 2D точности при интеграции TCAS: {improvement_2d:.1f}%\n")
            f.write(f"• Улучшение 3D точности при интеграции TCAS: {improvement_3d:.1f}%\n")
        
        if 'Плотный трафик' in data:
            f.write(f"• В условиях плотного трафика TCAS доступен {df_comparison.loc['Плотный трафик', 'tcas_availability']:.1f}% времени\n")
        
        if 'Нормальные условия' in data:
            f.write(f"• В нормальных условиях средняя 2D ошибка: {df_comparison.loc['Нормальные условия', 'mean_2d_error']:.1f} м\n")
        
        f.write("\n4. ПРАКТИЧЕСКИЕ РЕКОМЕНДАЦИИ\n")
        f.write("-" * 40 + "\n")
        f.write("• Интеграция TCAS в систему КОИ целесообразна в следующих случаях:\n")
        f.write("  - При эксплуатации в аэропортах с плотным воздушным движением\n")
        f.write("  - В условиях возможных помех или потери сигнала СНС\n")
        f.write("  - Для повышения отказоустойчивости навигационной системы\n")
        f.write("\n• Ограничения применения:\n")
        f.write("  - Эффективность зависит от количества доступных целей TCAS\n")
        f.write("  - Требуется калибровка систематических ошибок пеленга\n")
        f.write("  - Необходима дополнительная сертификация для применения в гражданской авиации\n")
        
        f.write("\n5. ПЕРСПЕКТИВЫ РАЗВИТИЯ\n")
        f.write("-" * 40 + "\n")
        f.write("• Интеграция с ADS-B для повышения точности определения координат\n")
        f.write("• Разработка алгоритмов машинного обучения для оптимальной фильтрации\n")
        f.write("• Использование в беспилотных летательных аппаратах\n")
        f.write("• Расширение функциональности для горизонтальных маневров\n")
    
    print("\nАнализ завершен!")
    print("Созданы следующие файлы:")
    print("- simulation_statistics.csv - таблица статистики")
    print("- analysis_report.txt - итоговый отчет")
    print("- PNG файлы с графиками")
    print("\nДля просмотра графиков используйте любой просмотрщик изображений.")

if __name__ == "__main__":
    main()