Maratona
========
## Container
```cpp
var.clear(); // Remove todos elementos

bool vazio = var.empty(); // Vazio?
unsigned in tamanho = var.size(); // Tamanho

for (; it != var.end(); it++) //Loop
```

## vector

```cpp
#include <vector>

// Declaração
vector<TIPO> var; 
vector<TIPO> var(QTD, VALOR);

var.push_back(VALOR);  // Adiciona elemento
var.pop_back(); // Remove elemento

vector<TIPO>::iterator it = var.begin(); //Iterador normal
vector<TIPO>::reverse_iterator rit = var.rbegin(); //Iterador reverso

var.insert(it, VALOR); // Insere na posição it
var.insert(it, QTD, VALOR); // Insere QTD VALORES na posição it
var.insert(it, ARRAY + 0, ARRAY + Tamanho); // Insere todos os valores de um array.

TIPO primeiro = var.front(); // Primeiro elemento
TIPO ultimo = var.back(); // Ultimo elemento
```

## map
```cpp
#include <map>
map<TIPO_KEY, TIPO_VALUE> var;

var.insert(make_pair(KEY, VALUE)); // Insere
var.erase(KEY); // Remove

TIPO_VALUE valor = var[KEY]; // Acessa mapa
map<int, int>::iterator key_value = var.find(KEY); // Busca elemento
bool existe = key_value != var.end(); // Existe?
```

## multimap
```cpp
#include <map>
multimap<TIPO_KEY, TIPO_VALUE> var;

var.insert(make_pair(KEY, VALUE)); // Insere
var.erase(KEY); // Remove todos os valores associados à KEY
multimap<TIPO_KEY, TIPO_VALUE>::iterator it = ...;
var.erase(it); // Remove uma entrada

multimap<int, int>::iterator key_value = var.find(KEY); // Busca alguma entrada
bool existe = key_value != var.end(); // Existe?

//Percorrimento das entradas de uma chave
for(multimap<TIPO_KEY, TIPO_VAL>::iterator it = var.lower_bound(KEY); it != var.upper_bound(KEY); it++){
	TIPO_KEY key = it->first;
	TIPO_VALUE value = it->second;
}
```

## set
```cpp
#include <set>
set<TIPO> var;

var.insert(ITEM); // Insere
var.erase(KEY); // Remove
bool existe = var.find(ITEM) != var.end(); // Contém?
```

## queue
```cpp
#include <queue>
queue<TIPO> var;

var.push(ITEM); // Insere
TIPO primeiro = var.front(); // Pega o primeiro sem remover
var.pop(); // Remove primeiro
```

## priority queue
```cpp
#include <queue>
priority_queue<TIPO> var; //Cria com comparador padrão Less<T>

class MyComparator{
	bool operator ()(const TIPO & v1, const TIPO & v2){...}
}

priority_queue<TIPO, std::vector<TIPO>, MyComparator> var; //Cria com comparador personalizado

var.push(ITEM); // Insere
TIPO primeiro = var.top(); // Pega o primeiro sem remover
var.pop(); // Remove primeiro
```


## stream
```cpp
istream& operator>>(istream &input, TIPO &p) {
  input >> p.VALOR1 >> p.VALOR2;
  return input;
}
ostream& operator<<(ostream &out, TIPO &p) {
  out << p.VALOR1 << " " << p.VALOR2;
  return out;
}


string a, b, c;
cin >> a;
cin.ignore(); // Discartar espaÃ§os
getline(cin, b);
cin >> c;
```

## sstream
```cpp
#include <sstream>

stringstream ss;
stringstream ss("text");

ss << "2"; // Input
int num;
ss >> num; // Output
string result = ss.str() // Tudo o que está no stream
```

## iostream e iomanip
```cpp
#include <iostream>

// input
char a, b, c;
istringstream ss("  123")'
ss >> skipws >> a >> b >> c;
cout << a << b << c; // '123'
iss.seekg(0);
ss >> noskipws >> a >> b >> c;
cout << a << b << c; // '  1'

// Float
cout.precision(5); 
cout << f; // 3.14159
cout << fixed << n; // 2006.00000
cout << scientific << n; // 2.00600e+003

// Base
cout << uppercase << hex << n; //4D
cout << nouppercase << hex << n; // 4d
cout << showbase << hex << n; //0x4d
cout << dec << n;
cout << oct << n;

// Bool
cout << boolalpha << b; // true
cout << noboolalpha << b; // 1

// Justify
cout.width(6); cout << internal << n; // "-   77"
cout.width(6); cout << left << n;     // "-77   "
cout.width(6); cout << right << n;    // "   -77"

#include <iomanip>

// Float
cout << setprecision(5) << f; // 3.14159

// Base
cout << setbase(16) << n; // 4d

// Justify
cout << setw(6) << n; // "   -77"

// Fill
cout << setfill('x') << setw(6) << n; // "xxx-77"

```


# TODO DOCS
- deque
- algorithm
- functional
- utility
- string
- limits
- complex
- cmath
 
# TODO ALG
- bfs
- djasktra
- bellman-ford
- floyd-marshall
- kruskal
- prim
- lcs
- lis
- modular_pow
