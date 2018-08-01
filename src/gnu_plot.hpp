#ifndef GNU_PLOT_HPP
#define GNU_PLOT_HPP
#include <string>
#include <cstdio>
class PlotUtils
{
public:
    virtual ~PlotUtils()
    {
        close();
    }

    bool is_opened() const
    {
        return m_pipe != NULL;
    }

    void open()
    {
        close();
        m_pipe = gnuplot_open("gnuplot", "w");
    }
    void close()
    {
        if  (is_opened())
        {
            gnuplot_close(m_pipe);
            m_pipe = NULL;
        }
    }
    void command(const std::string &cmd)
    {
        if (is_opened() && !cmd.empty())
        {
            std::fprintf(m_pipe, "%s\n", cmd.c_str());
            std::fflush(m_pipe);
        }
    }

private:
    FILE* m_pipe = NULL;

    static FILE* gnuplot_open(const char *command, const char *mode)
    {
        return popen(command, mode);
    }
    static int gnuplot_close(FILE *pipe)
    {
        return  pclose(pipe);
    }
};

#endif // GNU_PLOT_HPP
