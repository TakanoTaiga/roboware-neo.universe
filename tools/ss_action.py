import argparse

def read_csv(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    return lines

def parse_data(lines):
    results = []
    targets = []
    for i in range(0, len(lines), 2):
        result_values = list(map(float, lines[i].strip().split(',')[1:]))
        target_values = list(map(float, lines[i+1].strip().split(',')[1:]))
        results.append(result_values)
        targets.append(target_values)
    return results, targets

def calculate_errors(results, targets):
    errors = []
    for result, target in zip(results, targets):
        error = [abs(r - t) for r, t in zip(result, target)]
        errors.append(error)
    return errors

def write_markdown_tables(results, targets, errors, output_file, tittle):
    with open(output_file, 'w') as file:
        # x座標の表
        file.write("## Result: " + str(tittle) + "\n")
        file.write("### x座標の誤差\n")
        file.write("| Index | x座標 (Result) | x座標 (Target) | x座標 (Error) |\n")
        file.write("| --- | --- | --- | --- |\n")
        for i, (result, target, error) in enumerate(zip(results, targets, errors)):
            file.write(f"| {i+1} | {result[0]:.2f} | {target[0]:.2f} | {error[0]:.2f} |\n")
        
        # y座標の表
        file.write("\n### y座標の誤差\n")
        file.write("| Index | y座標 (Result) | y座標 (Target) | y座標 (Error) |\n")
        file.write("| --- | --- | --- | --- |\n")
        for i, (result, target, error) in enumerate(zip(results, targets, errors)):
            file.write(f"| {i+1} | {result[1]:.2f} | {target[1]:.2f} | {error[1]:.2f} |\n")
        
        # 角度の表
        file.write("\n### 角度の誤差\n")
        file.write("| Index | 角度 (Result) | 角度 (Target) | 角度 (Error) |\n")
        file.write("| --- | --- | --- | --- |\n")
        for i, (result, target, error) in enumerate(zip(results, targets, errors)):
            file.write(f"| {i+1} | {result[2]:.2f} | {target[2]:.2f} | {error[2]:.2f} |\n")

def main():
    parser = argparse.ArgumentParser(description='Process some files.')
    parser.add_argument('output_file', type=str, help='The output file name')
    parser.add_argument('tittle', type=str, help='The tittle')
    args = parser.parse_args()

    input_file = 'rw.log'
    output_file = args.output_file
    
    lines = read_csv(input_file)
    results, targets = parse_data(lines)
    errors = calculate_errors(results, targets)
    write_markdown_tables(results, targets, errors, output_file, args.tittle)
    
    print(f"結果が'{output_file}'ファイルに出力されました。")

if __name__ == "__main__":
    main()
