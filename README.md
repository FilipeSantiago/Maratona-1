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

## deque << vector
```cpp
#include <deque>

// Declaração
deque<TIPO> var;
var.push_front(VALOR); // Adiciona no inicio
var.pop_front(); // Remove do inicio

// Todas outras operações de vector
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

## algorithm
```cpp
#include <algorithm>

// Básico
void imprime(int i) {cout << i << endl;}
bool menor(int i,int j) { return i<j; }
bool igual(int i,int j) { return i==j; }
bool par(int i){ return i%2==0; }
vector<int> vet, vet2;

// Percorre aplicando função
for_each(vet.begin(), vet.end(), imprime);

// Busca
vector<int>::iterator it;
it = find(vet.begin(), vet.end(), VALOR); // linear
it = find_if(vet.begin(), vet.end(), par); // linear com condição
it = binary_search(vet.begin(), vet.end(), VALOR, menor); // binária
it = search(vet.begin(), vet.end(), vet2.begin(), vet2.end(), igual); // sublista
it = find_end(vet.begin(), vet.end(), vet2.begin(), vet2.end(), igual); // sublista no final
it = find_first_of(vet.begin(), vet.end(), vet2.begin(), vet2.end(), igual); // um dos resultados
it = adjacent_find(vet.begin(), vet.end(), vet2.begin(), vet2.end(), igual); // elementos repetidos
it = search_n(vet.begin(), vet.end(), QTD, VALOR); // QTD VALORES seguidos

pair<vector<int>::iterator,vector<int>::iterator> par;
par = mismatch(vet.begin(), vet.end(), vet2.begin()); // Primeira diferença

// Contar
int c = count(vet.begin(), vet.end(), 10);
int c = count_if(vet.begin(), vet.end(), par);

// Igualgdade
bool igual = equal(vet.begin(), vet.end(), vet2.begin(), igual);

// Copiar
vet2.resize(vet.size());
copy(vet.begin(), vet.end(), vet2.begin()); 
copy_backward(vet.begin(), vet.end(), vet2.end()); // inverso

// Swap
swap(vet, vet2);


// Continuar
http://www.cplusplus.com/reference/algorithm/


```

## cmath
```cpp
//Funções trigonométricas - em radianos
double angle = 3.14;
double v = sin(angle) + cos(angle) + tan(angle) +
	 acos(1.0) + // [0, pi]
	 asin(1.0) + // [-pi/2,+pi/2] 
	 atan(1.0);  // [-pi/2,+pi/2]
double x,y;
v = atan2(y, x);  // [-pi,+pi]

v = sinh(angle) + cosh(angle) + tanh(angle); // Funções hiperbólicas

double PI = 355.0 / 113.0; //Aproximação de Pi

double base = 3, expoent = 10;
v = sqrt(25);           // Raiz quadrada
v = pow(base, expoent); // 3^10
v = exp(expoent);       // e^10
v = log(1234);          // Logaritmo nepteriano
v = log10(1234);        // Log na base 10

int integer;
double frac = modf(13.4, &integer); //Separa as partes inteira e decimal.

v = log2(1024); // Log na base 2, C++11 apenas.

v = ceil(12.3); // 13
v = floor(12.3); // 12
v = abs(-3.4); // 3.4, Módulo

v = fmod(5.3, 2.0); // 1.3, resto fracionário da divisão inteira

// Constantes
v = NAN;
v = HUGE_VAL; // ~= infinito
v = INFINITY; // C++11
```

## C++ Básico
```cpp

// Classe - private por padrão
class Simple { 
  int i_;
public:
  Simple(int i) : i_(i) {}; // Constructor
  ~Simple(); // Destructor
  Simple(const Simple &); // Copy Constructor
  Simple & operator=(const Simple &); // Assignment Operator
  void print_i(); // método
};

// Struct - public por padrão
struct Simple { 
  Simple(int i) : i_(i) {}; // construtor
  void print_i(); // método
private:
  int i_;
};

void Simple::print_i() {
  cout << i_ << endl;
}

// Alocação dinâmica
int * a = new int [3];
delete [] a;

Simplt * s = new Simple;
delete s;

// Parametros:
void foo(int a = 2); // Valor padrão
void foo

// Try catch
try {
  //código
  throw 2;
} catch (RangeError &re) {
  // específica
} catch (int &i) {
  // Exceção 2
} catch (...) {
  // Qualquer exceção
}

vor foo(); // pode lançar qualquer exceção
vor foo() throw(); // não lança exceção
vor foo() throw(int); // só lança exceção int


// Operadores
TIPO operator()(TIPO param1, TIPO param2, ...) // function call
TIPO& operator++() // ++valor
TIPO operator++(int) // valor++
TIPO& operator+=(const TIPO& outro) // +=
TIPO operator+(TIPO primeiro, const TIPO& segundo) // primeiro + segundo
bool operator<(const TIPO& primeiro, const TIPO& segundo) // p < s; único necessário
bool operator==(const TIPO& primeiro, const TIPO& segundo) // p == s; único necessário
const TIPO& operator[](size_t idx) const // var[idx]


```



# TODO DOCS
- algorithm
- functional
- utility
- string
- limits
- complex
 
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
