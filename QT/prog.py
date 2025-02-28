def generate_numbers(target_sum, num_digits):
    if target_sum < 0:
        raise Exception("Invalid target sum")
    elif num_digits == 3:
        end = 999
    elif num_digits == 4:
        end = 9999
    else:
        raise Exception("Invalid number of digits")

    valid_numbers = []
    for number in range(0, end + 1):
        if sum(int(digit) for digit in str(number)) == target_sum:
            valid_numbers.append(str(number).zfill(num_digits))

    return valid_numbers, len(valid_numbers)

def to_string_numbers(numbers):
    return '\n'.join([str(x) for x in numbers])