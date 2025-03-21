#ifndef WINT_H
#define WINT_H
#include <bitset>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

static std::vector<int> double_vec(std::vector<int> vec)
{
    std::vector<int> result(vec.size(), 0);
    for (int i = 0; i < vec.size(); ++i)
    {
        result[i] += vec[i] * 2;
        if (result[i] > 9)
        {
            result[i] -= 10;
            result[i + 1] += 1;
        }
    }
    return result;
}

template <int NBits, bool Signed = true>
class WInt
{
public:
    WInt(int n = 0) : data_(n) {};
    WInt(const std::bitset<NBits>& n) : data_(n) {};

    template <int NBits2, bool Signed2>
    WInt(const WInt<NBits2, Signed2>& other)
    {
        const std::bitset<NBits2>& othdata = other.get_data();
        for (int i = NBits; i < NBits2; ++i)
        {
            if (othdata[i])
            {
                throw std::runtime_error("Cannot fit the value into the current bitset.");
            }
        }

        for (int i = 0; i < NBits; i++)
        {
            data_[i] = (i < NBits2) ? othdata[i] : 0;
        }

        if (Signed && !Signed2)
        {
            data_[NBits - 1] = 0;
        }
    }

    WInt operator-() const
    {
        if (!Signed)
        {
            throw std::runtime_error("Negation is not supported for unsigned integers.");
        }
        return WInt(sum(~data_, std::bitset<NBits>(1)));
    }

    WInt& operator+=(const WInt& other)
    {
        data_ = sum(data_, other.data_);
        return *this;
    }
    WInt& operator-=(const WInt& other)
    {
        data_ = subtract(data_, other.data_);
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
                result = sum(result, temp << i);
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

        for (int i = NBits - 1; i >= 0; i--)
        {
            rem = rem << 1;
            if (data_[i] == 1)
                rem = rem | one;

            if (is_greater_than(rem, other.data_, 1))
            {
                rem = subtract(rem, other.data_);
                res[i] = 1;
            }
        }
        data_ = res;
        return *this;
    }

    bool operator<(const WInt& other) const
    {
        return is_greater_than(other.data_, data_, 0);
    }
    bool operator>(const WInt& other) const
    {
        return is_greater_than(data_, other.data_, 0);
    }
    bool operator<=(const WInt& other) const
    {
        return is_greater_than(other.data_, data_, 1);
    }
    bool operator>=(const WInt& other) const
    {
        return is_greater_than(data_, other.data_, 1);
    }

    bool operator==(const WInt& other) const
    {
        return (data_ ^ other.data_) == 0;
    }
    bool operator!=(const WInt& other) const
    {
        return (data_ ^ other.data_) != 0;
    }

    const std::bitset<NBits>& get_data() const
    {
        return data_;
    }
    
    bool is_negate() const
    {
        return (Signed && data_[NBits - 1]) ? true : false;
    }
    friend std::ostream& operator<<(std::ostream& oss, const WInt& x)
    {
        auto res = x.to_vector();
        int i = res.size() - 1;
        while (res[i] == 0)
        {
            i--;
            if (i < 0)
            {
                return oss << "0";
            }
        }
        if (x.is_negate())
        {
            oss << "-";
        }
        std::string str = "";
        for (int j = i; j >= 0; j--)
        {
            str += std::to_string(res[j]);
            if (j % 3 == 0 && j != 0)
            {
                str += "'";
            }
        }
        return oss << str;
    }
   
    std::string print_binary() const
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

private:
    std::bitset<NBits> data_;

    static std::bitset<NBits> sum(const std::bitset<NBits>& s1, const std::bitset<NBits>& s2)
    {
        std::bitset<NBits> s;
        std::bitset<NBits> a(s1);
        std::bitset<NBits> b(s2);
        while (b != 0)
        {
            s = a ^ b;
            b = (a & b) << 1;
            a = s;
        }
        return a;
    }
    static std::bitset<NBits> subtract(const std::bitset<NBits>& s1, const std::bitset<NBits>& s2)
    {
        std::bitset<NBits> s;
        std::bitset<NBits> a(s1);
        std::bitset<NBits> b(s2);
        while (b != 0)
        {
            s = a ^ b;
            b = (~a & b) << 1;
            a = s;
        }
        return a;
    }

    bool is_greater(const std::bitset<NBits>& a, const std::bitset<NBits>& b, bool need_equal) const
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

    bool is_greater_than(const std::bitset<NBits>& a, const std::bitset<NBits>& b, bool need_equal) const
    {
        if (!Signed || (Signed && !a[NBits - 1] && !b[NBits - 1]))
        {
            return is_greater(a, b, need_equal);
        }
        else if (a[NBits - 1] && b[NBits - 1])
        {
            return is_greater(b, a, need_equal);
        }
        return !a[NBits - 1];
    }

    std::vector<int> to_vector() const
    {
        int n = (std::floor(NBits * std::log10(2))) + 1;
        std::vector<int> result(n, 0);
        std::vector<int> pow_of2(n, 0);
        pow_of2[0] = 1;

        auto data = data_;
        if (Signed && data_[NBits - 1])
        {
            data = sum(~data, std::bitset<NBits>(1));
        }
        for (int i = 0; i < NBits; ++i)
        {
            if (data[i])
            {
                for (int j = 0; j < pow_of2.size(); ++j)
                {
                    result[j] += pow_of2[j];
                    while (result[j] > 9)
                    {
                        result[j] -= 10;
                        result[j + 1] += 1;
                    }
                }
            }
            pow_of2 = double_vec(pow_of2);
        }
        return result;
    }
};

#endif