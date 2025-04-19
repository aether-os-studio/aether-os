#include <libsyscall.h>
#include <stdio.h>
#include "pl_readline.h"

int getc()
{
    int ch = (int)getchar();
    switch (ch)
    {
    case '\b':
        return PL_READLINE_KEY_BACKSPACE;
    case '\t':
        return PL_READLINE_KEY_TAB;
    case '\n':
        return PL_READLINE_KEY_ENTER;
    case -1:
        return PL_READLINE_KEY_UP;
    case -2:
        return PL_READLINE_KEY_DOWN;
    case -3:
        return PL_READLINE_KEY_LEFT;
    case -4:
        return PL_READLINE_KEY_RIGHT;
    default:
        return ch;
    }
}

void putc(int ch)
{
    printf("%c", (char)ch);
}

void flush()
{
}

static void handle_tab(char *buf, pl_readline_words_t words)
{
    pl_readline_word_maker_add("ls", words, true, ' ');

    if (buf[0] != '/' && strlen(buf))
    {
        return;
    }
    char *s = malloc(strlen(buf) + 2);
    memcpy(s, buf, strlen(buf) + 1);
    if (strlen(s))
    {
        for (isize i = strlen(s); i >= 0; i--)
        {
            if (s[i] == '/')
            {
                s[i + 1] = '\0';
                break;
            }
        }
    }
    else
    {
        s[0] = '/';
        s[1] = '\0';
    }

    int fd = open(s, 0, 0);
    dirent_t dents[128];
    int num = getdents(fd, dents, 128);
    if (num < 0)
    {
        close(fd);
        return;
    }

    for (int i = 0; i < num; i++)
    {
        char *new_path = pathacat(s, dents[i].name);
        pl_readline_word_maker_add(new_path, words, false, dents[i].type == file_dir ? '/' : ' ');
    }

    close(fd);
}

int list_files(char *path)
{
    int fd = open((const char *)path, 0, 0);
    dirent_t dents[128];
    int num = getdents(fd, dents, 128);
    if (num < 0)
    {
        return num;
    }

    for (int i = 0; i < num; i++)
    {
        printf("%s\t ", dents[i].name);
    }
    printf("\n");

    close(fd);

    return 0;
}

static int shell_exec(char *path, const char *command)
{
    if (!strlen(command))
        return 0;

    int retcode = 0;
    if (!strcmp(command, "ls"))
    {
        retcode = list_files(path);
    }

    return retcode;
}

int main()
{
    printf("shell is running\n");

    char *path = malloc(1024);
    memset(path, 0, 1024);
    sprintf(path, "/");

    pl_readline_t readln = pl_readline_init(getc, putc, flush, handle_tab);

    char prompt[256];
    while (true)
    {
        sprintf(prompt, "\033[1;32mroot\033[m:\033[1;36m%s\033[m#\033[1;33m ", path);
        const char *line = pl_readline(readln, prompt);
        printf("\033[m");
        shell_exec(path, line);
    }

    while (1)
    {
        __asm__ __volatile__("pause");
    }
}
