#ifndef WINT_H
#define WINT_H
#include <bitset>
#include <iostream>
#include <string>

template <int NBits, bool Signed = true>
class WInt
{
public:
    WInt(int n = 0)
    {
        data_ = n;
    }

    template <int NBits2, bool Signed2>
    WInt(const WInt<NBits2, Signed2>& other)
    {
        auto othdata = other.get_data();

        for (int i = 0; i < NBits; i++)
        {
            data_[i] = (i < NBits2) ? othdata[i] : 0;
        }

        if (Signed && !Signed2)
        {
            data_[NBits - 1] = 0;
        }
    }

    WInt& operator+=(const WInt& other)
    {
        std::bitset<NBits> plus1 = other.data_;
        data_ = sum(data_, plus1, 0);
        return *this;
    }

    WInt& operator-=(const WInt& other)
    {
        std::bitset<NBits> plus1 = other.data_;
        data_ = sum(data_, plus1, 1);
        return *this;
    }

    WInt& operator*=(const WInt& other)
    {
        std::bitset<NBits> result(0);
        std::bitset<NBits> temp = data_;

        for (size_t i = 0; i < NBits; ++i)
        {
            if (other.data_[i])
            {
                result = sum(result, temp << i, 0);
            }
        }

        data_ = result;
        return *this;
    }

    WInt& operator/=(const WInt& other)
    {
        if (other.data_ == 0)
        {
            throw std::runtime_error("Division by zero");
        }
        std::bitset<NBits> res(0);
        std::bitset<NBits> rem(0);

        const std::bitset<NBits> one(1);
        const std::bitset<NBits> zero(0);

        for (int I = NBits - 1; I >= 0; I--)
        {
            rem = rem << 1;
            if (data_[I] == 1)
                rem = rem | one;
            else
                rem = rem | zero;

            if (is_more_then(rem, other.data_, 1))
            {
                rem = sum(rem, other.data_, 1);
                res[I] = 1;
            }
        }
        data_ = res;
        return *this;
    }

    bool operator<(const WInt& other) const
    {
        return is_more_then(other.data_, data_, 0);
    }
    bool operator>(const WInt& other) const
    {
        return is_more_then(data_, other.data_, 0);
    }
    bool operator<=(const WInt& other) const
    {
        return is_more_then(other.data_, data_, 1);
    }
    bool operator>=(const WInt& other) const
    {
        return is_more_then(data_, other.data_, 1);
    }
    bool operator==(const WInt& other) const
    {
        return (data_ ^ other.data_) == 0;
    }

    std::string print_binary()
    {
        std::string str = data_.to_string();
        std::string res = "";
        for (size_t i = 0; i < str.size(); ++i)
        {
            if (i > 0 && (str.size() - i) % 3 == 0)
            {
                res += "'";
            }
            res += str[i];
        }
        return res;
    }

    friend std::ostream& operator<<(std::ostream& oss, const WInt& x)
    {
        long long decimalValue = x.to_decimal();

        std::string str = std::to_string(decimalValue);
        for (size_t i = 0; i < str.size(); ++i)
        {
            if (i > 0 && (str.size() - i) % 3 == 0)
            {
                oss << "'";
            }
            oss << str[i];
        }
        return oss;
    }

    std::bitset<NBits> get_data() const
    {
        return data_;
    }

private:
    std::bitset<NBits> data_;
    std::bitset<NBits> sum(std::bitset<NBits> a, std::bitset<NBits> b, bool is_substract)
    {
        std::bitset<NBits> s;
        while (b != 0)
        {
            s = a ^ b;
            if (is_substract)
                b = (~a & b) << 1;
            else
                b = (a & b) << 1;
            a = s;
        }
        return a;
    }

    bool compare_positive(const std::bitset<NBits>& a, const std::bitset<NBits>& b, bool need_equal) const
    {
        for (int I = NBits - 1; I >= 0; I--)
        {
            if (a[I] != b[I])
            {
                return a[I] == 1;
            }
        }
        return need_equal;
    }

    bool is_more_then(const std::bitset<NBits>& a, const std::bitset<NBits>& b, bool need_equal) const
    {
        bool is_a_negative = a[NBits - 1];
        bool is_b_negative = b[NBits - 1];

        if (!is_a_negative && !is_b_negative)
        {
            return compare_positive(a, b, need_equal);
        }
        else if (is_a_negative && is_b_negative)
        {
            return compare_positive(b, a, need_equal);
        }

        return !is_a_negative;
    }

    long long to_decimal() const
    {
        long long result = 0;
        for (int i = 0; i < NBits; ++i)
        {
            if (data_[i])
            {
                result += (1LL << i);
            }
        }
        if (Signed && data_[NBits - 1])
        {
            result -= (1LL << NBits);
        }
        return result;
    }
};

#endif