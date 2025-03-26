#ifndef WINT_H
#define WINT_H
#include <bitset>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

template <int NBits, bool Signed = true>
class WInt
{
public:
    WInt(int n = 0) : data_(n) {};
    WInt(const std::bitset<NBits>& n) : data_(n) {};

    template <int NBits2, bool Signed2>
    WInt(const WInt<NBits2, Signed2>& other)
    {
        std::bitset<NBits2> othdata = other.get_data();
        if constexpr (Signed2)
        {
            if (othdata[NBits2 - 1])
            {
                if constexpr (!Signed)
                    throw std::runtime_error("Cannot fit the negative value into the positive bitset.");
                else
                    othdata = convert(othdata);
            }
        }
        if constexpr (NBits2 > NBits)
        {
            if ((othdata >> NBits).any())
                throw std::runtime_error("Cannot fit the value into the current bitset.");
        }

        for (int i = 0; i < NBits; i++)
        {
            data_[i] = (i < NBits2) ? othdata[i] : 0;
        }

        if (other.get_data()[NBits2 - 1])
        {
            data_ = convert(data_);
        }
    }

    WInt operator-() const
    {
        if constexpr (!Signed)
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

            if (compare(rem, other.data_) >= 0)
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
        return compare(data_, other.data_) < 0;
    }
    bool operator>(const WInt& other) const
    {
        return compare(data_, other.data_) > 0;
    }
    bool operator<=(const WInt& other) const
    {
        return compare(data_, other.data_) <= 0;
    }
    bool operator>=(const WInt& other) const
    {
        return compare(data_, other.data_) >= 0;
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
    
    bool is_negative() const
    {
        if constexpr (Signed)
        {
            return data_[NBits - 1];
        }
        else
        {
            return false;
        }
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
        if (x.is_negative())
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

    static constexpr int n_decimal_digit = (int(NBits * 0.30103)) + 1;
   
    template <int NBits2>
    static std::bitset<NBits2> sum(std::bitset<NBits2> a, std::bitset<NBits2> b)
    {
        std::bitset<NBits2> s;
        while (b != 0)
        {
            s = a ^ b;
            b = (a & b) << 1;
            a = s;
        }
        return a;
    }
    static std::bitset<NBits> subtract(std::bitset<NBits> a, std::bitset<NBits> b)
    {
        std::bitset<NBits> s;
        while (b != 0)
        {
            s = a ^ b;
            b = (~a & b) << 1;
            a = s;
        }
        return a;
    }

    int positive_compare(const std::bitset<NBits>& a, const std::bitset<NBits>& b) const 
    {
        for (int i = NBits - 1; i >= 0; i--)
        {
            if (a[i] != b[i])
            {
                return a[i] ? 1 : -1;
            }
        }
        return 0;
    }

    int compare(const std::bitset<NBits>& a, const std::bitset<NBits>& b) const
    {
        if (!Signed || (Signed && !a[NBits - 1] && !b[NBits - 1]))
        {
            return positive_compare(a, b);
        }
        else if (a[NBits - 1] && b[NBits - 1])
        {
            return positive_compare(b, a);
        }
        return (!a[NBits - 1]) ? 1 : -1;
    }

    std::vector<int> to_vector() const
    {
        std::vector<int> result(n_decimal_digit, 0);
        std::vector<int> pow_of2(n_decimal_digit, 0);
        pow_of2[0] = 1;

        std::bitset<NBits> data(data_);
        if constexpr (Signed)
        {
            if (data[NBits - 1])
                data = convert(data_);
        }
        for (int i = 0; i < NBits; ++i)
        {
            if (data[i])
            {
                result = vec_sum(result, pow_of2);
            }
            pow_of2 = vec_sum(pow_of2, pow_of2);
        }
        return result;
    }

    static std::vector<int> vec_sum(const std::vector<int>& a, const std::vector<int>& b)
    {
        std::vector<int> res(a);
        for (int j = 0; j < b.size(); ++j)
        {
            if (j == res.size()) res.push_back(0);
            res[j] += b[j];
            while (res[j] > 9)
            {
                if (j + 1 == res.size()) res.push_back(0);
                res[j] -= 10;
                res[j + 1] += 1;
            }
        }
        return res;
    }

    template <int NBits2>
    static std::bitset<NBits2> convert(const std::bitset<NBits2>& x)
    {
        std::bitset<NBits2> data(x);
        data = sum(~data, std::bitset<NBits2>(1));
        return data;
    }
};

#endif