from itertools import zip_longest
from math import inf
import math
import numpy as np
import shutil

class UI():
    console_width = 80
    console_height = 24
    
    @staticmethod
    def read_particle(prompt, str_options, dtype=str, interval=[-inf,inf]):
        ''' 
        Read particle from user from a given dtype or from a str_options list

        Parameters
        ----------
        prompt : `str`
            prompt to show user before reading input
        str_options : `list`
            list of str options (in addition to dtype if dtype is not str)
        dtype : `class`
            if dtype is str, then user must choose a value from str_options, otherwise it can also send a dtype value
        interval : `list`
            [>=min,<max] interval for numeric dtypes
        
        Returns
        -------
        choice : `int` or dtype
            index of str_options (int) or value (dtype)
        is_str_option : `bool`
            True if `choice` is an index from str_options
        '''
        # Check if user has no choice
        if dtype is str and len(str_options) == 1:
            print(prompt, str_options[0], sep="")
            return 0, True
        elif dtype is int and interval[0] == interval[1]-1:
            print(prompt, interval[0], sep="")
            return interval[0], False

        while True:
            inp = input(prompt)
            if inp in str_options: 
                return str_options.index(inp), True

            if dtype is not str:
                try:
                    inp = dtype(inp)
                    if inp >= interval[0] and inp < interval[1]:
                        return inp, False
                except:
                    pass
            
            print("Error: illegal input! Options:", str_options, f" or  {dtype}" if dtype != str else "")

    @staticmethod
    def read_int(prompt, min, max):
        ''' 
        Read int from user in a given interval
        :param prompt: prompt to show user before reading input
        :param min: minimum input (inclusive)
        :param max: maximum input (exclusive)
        :return: choice
        '''
        while True:
            inp = input(prompt)
            try:
                inp = int(inp)
                assert inp >= min and inp < max
                return inp
            except:
                print(f"Error: illegal input! Choose number between {min} and {max-1}")

    @staticmethod
    def print_table(data, titles=None, alignment=None, cols_width=None, cols_per_title=None, margins=None, numbering=None, prompt=None):
        '''
        Print table
        
        Parameters
        ----------
        data : `list`
            list of columns, where each column is a list of items
        titles : `list`
            list of titles for each column, default is `None` (no titles)
        alignment : `list`
            list of alignments per column (excluding titles), default is `None` (left alignment for all cols)
        cols_width : `list`
            list of widths per column, default is `None` (fit to content)
            Positive values indicate a fixed column width
            Zero indicates that the column will fit its content
        cols_per_title : `list`
            maximum number of subcolumns per title, default is `None` (1 subcolumn per title)
        margins : `list`
            number of added leading and trailing spaces per column, default is `None` (margin=2 for all columns)
        numbering : `list`
            list of booleans per columns, indicating whether to assign numbers to each option
        prompt : `str`
            the prompt string, if given, is printed after the table before reading input

        Returns
        -------
        index : `int`
            returns global index of selected item (relative to table)
        col_index : `int`
            returns local index of selected item (relative to column)
        column : `int`
            returns number of column of selected item (starts at 0)
        * if `numbering` or `prompt` are `None`, `None` is returned
        

        Example
        -------
        titles = ["Name","Age"]
        data = [[John,Graciete], [30,50]]
        alignment = ["<","^"]               # 1st column is left-aligned, 2nd is centered
        cols_width = [10,5]                # 1st column's width=10, 2nd column's width=5
        margins = [3,3]                    
        numbering = [True,False]           # prints: [0-John,1-Graciete][30,50]
        prompt = "Choose a person:"
        '''
        
        #--------------------------------------------- parameters
        cols_no = len(data)

        if alignment is None:
            alignment = ["<"]*cols_no

        if cols_width is None:
            cols_width = [0]*cols_no

        if numbering is None:
            numbering = [False]*cols_no
            any_numbering = False
        else:
            any_numbering = True

        if margins is None:
            margins = [2]*cols_no

        # Fit column to content + margin, if required
        subcol = [] # subcolumn length and widths
        for i in range(cols_no):
            subcol.append([[],[]])
            if cols_width[i] == 0:
                numbering_width = 4 if numbering[i] else 0
                if cols_per_title is None or cols_per_title[i] < 2:
                    cols_width[i] = max([len(str(item))+numbering_width for item in data[i]]) + margins[i]*2
                else:
                    subcol[i][0] = math.ceil(len(data[i])/cols_per_title[i]) # subcolumn maximum length
                    cols_per_title[i] = math.ceil(len(data[i])/subcol[i][0]) # reduce number of columns as needed
                    cols_width[i] = margins[i]*(1+cols_per_title[i]) - (1 if numbering[i] else 0) # remove one if numbering, same as when printing
                    for j in range(cols_per_title[i]):
                        subcol_data_width = max([len(str(item))+numbering_width for item in data[i][j*subcol[i][0]:j*subcol[i][0]+subcol[i][0]]])
                        cols_width[i] += subcol_data_width     # add subcolumn data width to column width
                        subcol[i][1].append(subcol_data_width) # save subcolumn data width
                        
                if titles is not None: # expand to acomodate titles if needed
                    cols_width[i] = max(cols_width[i], len(titles[i]) + margins[i]*2  )

        if any_numbering:
            no_of_items=0
            cumulative_item_per_col=[0] # useful for getting the local index
            for i in range(cols_no):
                assert type(data[i]) == list, "In function 'print_table', 'data' must be a list of lists!"

                if numbering[i]:
                    data[i] = [f"{n+no_of_items:3}-{d}" for n,d in enumerate(data[i])]
                    no_of_items+=len(data[i])
                cumulative_item_per_col.append(no_of_items)

        table_width = sum(cols_width)+cols_no-1

        #--------------------------------------------- col titles
        print(f'{"="*table_width}')
        if titles is not None:
            for i in range(cols_no):
                print(f'{titles[i]:^{cols_width[i]}}', end='|' if i < cols_no - 1 else '')
            print()
            for i in range(cols_no):
                print(f'{"-"*cols_width[i]}', end='+' if i < cols_no - 1 else '')
            print()

        #--------------------------------------------- merge subcolumns
        if cols_per_title is not None:
            for i,col in enumerate(data):
                if cols_per_title[i] < 2:
                    continue
                for k in range(subcol[i][0]): # create merged items
                    col[k] = (" "*margins[i]).join( f'{col[item]:{alignment[i]}{subcol[i][1][subcol_idx]}}' 
                                                    for subcol_idx, item in enumerate(range(k,len(col),subcol[i][0])) )
                del col[subcol[i][0]:] # delete repeated items
        
        #--------------------------------------------- col items
        for line in zip_longest(*data):       
            for i,item in enumerate(line):
                l_margin = margins[i]-1 if numbering[i] else margins[i] # adjust margins when there are numbered options
                item = "" if item is None else f'{" "*l_margin}{item}{" "*margins[i]}' # add margins
                print(f'{item:{alignment[i]}{cols_width[i]}}', end='')
                if i < cols_no - 1:
                    print(end='|')
            print(end="\n")
        print(f'{"="*table_width}')

        #--------------------------------------------- prompt
        if prompt is None:
            return None

        if not any_numbering:
            print(prompt)
            return None

        index = UI.read_int(prompt, 0, no_of_items)

        for i,n in enumerate(cumulative_item_per_col):
            if index < n:
                return index, index-cumulative_item_per_col[i-1], i-1

        raise ValueError('Failed to catch illegal input')


    @staticmethod
    def print_list(data, numbering=True, prompt=None, divider=" | ", alignment="<", min_per_col=6):
        '''
        Print list - prints list, using as many columns as possible
        
        Parameters
        ----------
        data : `list`
            list of items
        numbering : `bool`
            assigns number to each option
        prompt : `str`
            the prompt string, if given, is printed after the table before reading input
        divider : `str`
            string that divides columns
        alignment : `str`
            f-string style alignment ( '<', '>', '^' )
        min_per_col : int
            avoid splitting columns with fewer items
        
        Returns
        -------
        item : `int`, item
            returns tuple with global index of selected item and the item object,
            or `None` (if `numbering` or `prompt` are `None`)

        '''
        
        WIDTH = shutil.get_terminal_size()[0]

        data_size = len(data)   
        items = []
        items_len = []

        #--------------------------------------------- Add numbers, margins and divider
        for i in range(data_size):
            number = f"{i}-" if numbering else ""
            items.append( f"{divider}{number}{data[i]}" )
            items_len.append( len(items[-1]) )

        max_cols = np.clip((WIDTH+len(divider)) // min(items_len),1,math.ceil(data_size/max(min_per_col,1))) # width + len(divider) because it is not needed in last col

        #--------------------------------------------- Check maximum number of columns, considering content width (min:1)
        for i in range(max_cols,0,-1):
            cols_width = []
            cols_items = []
            table_width = 0
            a,b = divmod(data_size,i)
            for col in range(i):
                start = a*col + min(b,col)
                end = start+a+(1 if col<b else 0)
                cols_items.append( items[start:end] )
                col_width = max(items_len[start:end])
                cols_width.append( col_width )
                table_width += col_width
            if table_width <= WIDTH+len(divider):
                break
        table_width -= len(divider)
        
        #--------------------------------------------- Print columns
        print("="*table_width)
        for row in range(math.ceil(data_size / i)):
            for col in range(i):
                content = cols_items[col][row] if len(cols_items[col]) > row else divider # print divider when there are no items
                if col == 0:
                    l = len(divider)
                    print(end=f"{content[l:]:{alignment}{cols_width[col]-l}}")  # remove divider from 1st col
                else:
                    print(end=f"{content    :{alignment}{cols_width[col]  }}") 
            print()  
        print("="*table_width)

        #--------------------------------------------- Prompt
        if prompt is None:
            return None

        if numbering is None:
            return None
        else:
            idx = UI.read_int( prompt, 0, data_size )
            return idx, data[idx]