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
    """Построение графика ошибок позиционирования (только 2D, так как высота определяется радиовысотомером)"""
    fig, ax = plt.subplots(1, 1, figsize=(12, 6))
    
    for df, label in zip(df_list, labels):
        if df is not None:
            ax.plot(df['timestamp'], df['error_2d'], label=label, linewidth=2, alpha=0.8)
    
    ax.set_xlabel('Время, с')
    ax.set_ylabel('2D ошибка, м')
    ax.set_title(f'{title} - 2D ошибка позиционирования')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
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

# Функция plot_trajectory_3d удалена - на посадке высота определяется радиовысотомером

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
    """Построение распределения ошибок (только 2D, так как высота определяется радиовысотомером)"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 6))
    
    for df, label in zip(df_list, labels):
        if df is not None:
            ax.hist(df['error_2d'], bins=30, alpha=0.6, label=label, density=True)
    
    ax.set_xlabel('2D ошибка, м')
    ax.set_ylabel('Плотность вероятности')
    ax.set_title(f'{title} - Распределение 2D ошибок')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig

def plot_cumulative_mean_error(df_list, labels, title, window=60):
    """Накопленное математическое ожидание 2D ошибки + глобальное среднее

    Использует накопленное среднее (cumulative mean) от начала до каждого момента времени.
    """
    fig, ax = plt.subplots(figsize=(12, 6))

    for df, label in zip(df_list, labels):
        if df is not None and len(df) > 0:
            # Сортируем по времени на всякий случай
            df_sorted = df.sort_values('timestamp').reset_index(drop=True)
            errors = df_sorted['error_2d']

            # Проверяем наличие валидных данных
            if errors.isna().all() or len(errors) == 0:
                print(f"Предупреждение: нет валидных данных для {label}")
                continue

            # Накопленное мат. ожидание (среднее от начала до текущего момента)
            cumulative_mean = errors.expanding(min_periods=1).mean()
            
            # Убеждаемся, что нет NaN значений
            if cumulative_mean.isna().any():
                print(f"Предупреждение: обнаружены NaN в накопленном среднем для {label}")
                cumulative_mean = cumulative_mean.ffill().fillna(0)
            
            ax.plot(df_sorted['timestamp'], cumulative_mean,
                    label=f'{label} (накопленное МО)', linewidth=2, alpha=0.9)

            # Глобальное среднее как горизонтальная линия
            global_mean = errors.mean()
            if not np.isnan(global_mean):
                ax.hlines(global_mean,
                          df_sorted['timestamp'].min(),
                          df_sorted['timestamp'].max(),
                          linestyles='dashed', linewidth=1.5, alpha=0.7,
                          label=f'{label} (глобальное среднее)')

    ax.set_xlabel('Время, с')
    ax.set_ylabel('Накопленное математическое ожидание 2D ошибки, м')
    ax.set_title(f'{title} - накопленное математическое ожидание 2D ошибки')
    ax.grid(True, alpha=0.3)
    ax.legend()

    plt.tight_layout()
    return fig

def plot_error_stats(df_list, labels, title):
    """Графики матожидания и СКО ошибок координат (2D) и скорости"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), sharex='col')
    
    metrics = [
        ('error_2d', '2D ошибка, м'),
        ('velocity_error', 'Ошибка скорости, м/с')
    ]
    
    for col, (column, ylabel) in enumerate(metrics):
        ax_mean = axes[0, col]
        ax_std = axes[1, col]
        
        for df, label in zip(df_list, labels):
            if df is not None and len(df) > 0 and column in df.columns:
                df_sorted = df.sort_values('timestamp').reset_index(drop=True)
                
                # Используем скользящее среднее вместо накопленного для более адекватного отображения
                # Окно 60 секунд (примерно 60 точек при шаге 1 секунда)
                window_size = 60
                mean_series = df_sorted[column].rolling(window=window_size, min_periods=1, center=False).mean()
                
                # СКО также вычисляем в скользящем окне
                # Используем ddof=1 для несмещенной оценки (стандартная формула для выборки)
                std_series = df_sorted[column].rolling(window=window_size, min_periods=2, center=False).std(ddof=1)
                # Для точек с недостаточным количеством данных СКО = 0
                std_series = std_series.fillna(0.0)
                
                ax_mean.plot(df_sorted['timestamp'], mean_series,
                             label=label, linewidth=2, alpha=0.9)
                ax_std.plot(df_sorted['timestamp'], std_series,
                            label=label, linewidth=2, alpha=0.9)
        
        ax_mean.set_ylabel(f'Мат. ожидание\n({ylabel})')
        ax_std.set_ylabel(f'СКО\n({ylabel})')
        ax_std.set_xlabel('Время, с')
        ax_mean.grid(True, alpha=0.3)
        ax_std.grid(True, alpha=0.3)
    
    axes[0, 0].set_title(f'{title} - матожидание ошибок')
    axes[1, 0].set_title(f'{title} - СКО ошибок')
    
    # Легенду выносим в верхний левый подграфик
    handles, legend_labels = axes[0, 0].get_legend_handles_labels()
    if handles:
        axes[0, 0].legend(handles, legend_labels)
    
    plt.tight_layout()
    return fig

def plot_position_errors_per_mode(data_dict, title_prefix):
    """Отдельные графики 2D ошибок позиционирования для каждого режима (высота определяется радиовысотомером)"""
    filename_suffix = {
        'С TCAS': 'with_tcas',
        'Без TCAS': 'without_tcas'
    }
    
    for label, df in data_dict.items():
        if df is None or len(df) == 0:
            continue
        
        fig, ax = plt.subplots(1, 1, figsize=(12, 6))
        
        df_sorted = df.sort_values('timestamp')
        ax.plot(df_sorted['timestamp'], df_sorted['error_2d'],
                 label='2D ошибка', linewidth=2, alpha=0.9)
        
        ax.set_xlabel('Время, с')
        ax.set_ylabel('2D ошибка, м')
        ax.set_title(f'{title_prefix} - {label} (2D)')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        plt.tight_layout()
        
        suffix = filename_suffix.get(label, 'mode')
        fig.savefig(f'position_errors_{suffix}.png', dpi=300, bbox_inches='tight')
        plt.close(fig)

def plot_position_component_errors(df_list, labels, title):
    """Декомпозиция 2D ошибки на компоненты: широта, долгота (высота определяется радиовысотомером)"""
    EARTH_RADIUS = 6371000.0  # м
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    
    component_names = [
        ('lat_error_m', 'Ошибка по широте, м'),
        ('lon_error_m', 'Ошибка по долготе, м'),
    ]
    
    for df, label in zip(df_list, labels):
        if df is None or len(df) == 0:
            continue
        
        df_sorted = df.sort_values('timestamp').copy()
        
        # Вычисляем ошибки в метрах
        lat_true_rad = np.deg2rad(df_sorted['true_lat'])
        dlat_rad = np.deg2rad(df_sorted['est_lat'] - df_sorted['true_lat'])
        dlon_rad = np.deg2rad(df_sorted['est_lon'] - df_sorted['true_lon'])
        
        df_sorted['lat_error_m'] = dlat_rad * EARTH_RADIUS
        df_sorted['lon_error_m'] = dlon_rad * EARTH_RADIUS * np.cos(lat_true_rad)
        
        for ax, (col, ylabel) in zip(axes, component_names):
            ax.plot(df_sorted['timestamp'], df_sorted[col],
                    label=label, linewidth=2, alpha=0.9)
            ax.set_ylabel(ylabel)
            ax.grid(True, alpha=0.3)
    
    axes[-1].set_xlabel('Время, с')
    axes[0].set_title(f'{title} - компоненты ошибки (широта/долгота)')
    
    # Общая легенда
    handles, legend_labels = axes[0].get_legend_handles_labels()
    if handles:
        axes[0].legend(handles, legend_labels)
    
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
    # Оставляем только те сценарии, которые реально используются:
    # - С TCAS
    # - Без TCAS
    files = {
        'С TCAS': 'results_with_tcas.csv',
        'Без TCAS': 'results_without_tcas.csv'
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
                                   ['Без TCAS', 'С TCAS'], 
                                   'Сравнение интеграции TCAS')
        fig1.savefig('position_error_comparison.png', dpi=300, bbox_inches='tight')
        plt.close(fig1)
        
        fig2 = plot_velocity_errors([data['С TCAS'], data['Без TCAS']], 
                                   ['Без TCAS', 'С TCAS'], 
                                   'Сравнение интеграции TCAS')
        fig2.savefig('velocity_error_comparison.png', dpi=300, bbox_inches='tight')
        plt.close(fig2)
        
        fig3 = plot_error_distribution([data['С TCAS'], data['Без TCAS']], 
                                      ['Без TCAS', 'С TCAS'], 
                                      'Сравнение интеграции TCAS')
        fig3.savefig('error_distribution_comparison.png', dpi=300, bbox_inches='tight')
        plt.close(fig3)
    
    # 2. Дополнительный график: накопленное матожидание 2D ошибки
    if 'С TCAS' in data and 'Без TCAS' in data:
        fig4 = plot_cumulative_mean_error(
            [data['С TCAS'], data['Без TCAS']],
            ['Без TCAS', 'С TCAS'],
            'Сравнение интеграции TCAS'
        )
        fig4.savefig('cumulative_mean_2d_error_comparison.png', dpi=300, bbox_inches='tight')
        plt.close(fig4)
        
        # 2b. Матожидание и СКО ошибок координат и скорости
        fig4b = plot_error_stats(
            [data['С TCAS'], data['Без TCAS']],
            ['Без TCAS', 'С TCAS'],
            'Сравнение интеграции TCAS'
        )
        fig4b.savefig('error_stats_comparison.png', dpi=300, bbox_inches='tight')
        plt.close(fig4b)
    
    # 3. Отдельные графики ошибок позиционирования по режимам
    plot_position_errors_per_mode(data, 'Ошибки позиционирования по режимам')
    
    # 4. Декомпозиция ошибки позиционирования на компоненты (широта/долгота)
    if 'С TCAS' in data and 'Без TCAS' in data:
        fig7 = plot_position_component_errors(
            [data['С TCAS'], data['Без TCAS']],
            ['Без TCAS', 'С TCAS'],
            'Сравнение интеграции TCAS'
        )
        fig7.savefig('position_error_components_comparison.png', dpi=300, bbox_inches='tight')
        plt.close(fig7)
    
    # 5. Доступность TCAS
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
            f.write(f"• Улучшение 2D точности при интеграции TCAS: {improvement_2d:.1f}%\n")
            f.write("• Примечание: высота определяется радиовысотомером, поэтому 3D ошибка не анализируется\n")
        
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