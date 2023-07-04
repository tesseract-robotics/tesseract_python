import re

def preprocess_docstrings(app, what, name, obj, options, lines):
    # Only process function and method docstrings
    if what not in ['function', 'method']:
        return
    

    missing_newline_regex2 = re.compile(r"[ \t]*\:(?:param|type|return|rtype|raises)")
    for i in range(0,len(lines)-1):
        line = lines[i]
        next_line = lines[i+1]

        if missing_newline_regex2.match(next_line):
            if len(line.strip()) > 0:
                lines.insert(i+1, "\n")
                print("Inserted newline after line: " + line)


def setup(app):
    app.connect('autodoc-process-docstring', preprocess_docstrings)