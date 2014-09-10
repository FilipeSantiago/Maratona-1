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

<div></div>

## set
```cpp
#include <set>
set<TIPO> var;

var.insert(ITEM); // Insere
var.erase(KEY); // Remove
bool existe = var.find(ITEM) != var.end(); // Contém?
```

<br><br><br><br>

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
<br><br><br><br>

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
int dobrar(int i) { return 2*i; }
int aleatorio() { return rand()%100; }
vector<int> vet, vet2, ordenado;
// Considere <param> como opcional

// Percorre aplicando função
for_each(vet.begin(), vet.end(), imprime);

// Busca
vector<int>::iterator it;
it = find(vet.begin(), vet.end(), VALOR); // linear
it = find_if(vet.begin(), vet.end(), par); // linear com condição
it = binary_search(ordenado.begin(), ordenado.end(), VALOR, <menor>); // binária
it = lower_bound(ordenado.begin(), ordenado.end(), VALOR, <menor>); // ponteiro para o primeiro VALOR
it = upper_bound(ordenado.begin(), ordenado.end(), VALOR, <menor>); // ponteiro para o primeiro numero depois de VALOR
it = search(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <igual>); // sublista
it = find_end(vet.begin(), vet.end(), vet2.begin(), vet2.end(), igual); // sublista no final
it = find_first_of(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <igual>); // um dos resultados
it = adjacent_find(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <igual>); // elementos repetidos
it = search_n(vet.begin(), vet.end(), QTD, VALOR); // QTD VALORES seguidos

pair<vector<int>::iterator,vector<int>::iterator> par;
par = mismatch(vet.begin(), vet.end(), vet2.begin()); // Primeira diferença
par = equal_range(ordenado.begin(), ordenado.end(), VALOR, <menor>); // Subconjunto de VALORES

// Contar
int c = count(vet.begin(), vet.end(), 10);
int c = count_if(vet.begin(), vet.end(), par);

// Igualgdade
bool igual = equal(vet.begin(), vet.end(), vet2.begin(), <igual>);

// Copiar
vet2.resize(vet.size());
copy(vet.begin(), vet.end(), vet2.begin()); 
copy_backward(vet.begin(), vet.end(), vet2.end()); // inverso

// Swap
swap(vet, vet2); // Swap variavel
swap_ranges(vet.begin() + 1, vet.end() - 1, vet2.begin()); // Swap parte do vetor
iter_swap(vet.begin(), vet2.begin() + 1); // Swap dentro de vetor.

// Transform
vet2.resize(vet.size());
transform(vet.begin(), vet.end(), vet2.begin(), dobrar); // mapeia dobrar para cada elemento de vet
transform(vet.begin(), vet.end(), vet2.begin(), result.begin(), soma); // mapeia soma para cada para <vet[i], vet2[i]>

// Substituir elementos
replace(vet.begin(), vet.end(), ORIGINAL, DESTINO);
replace_if(vet.begin(), vet.end(), par, DESTINO);
replace_copy(vet.begin(), vet.end(), vet2.begin(), ORIGINAL, DESTINO); // Resultado em vet2
replace_copy_if(vet.begin(), vet.end(), vet2.begin(), par, DESTINO); // Resultado em vet2

// Preencher
fill(vet.begin(), vet.begin() + 3, VALOR); // VALOR VALOR VALOR vet[4] vet[5] ...
fill_n(vet.begin(), 3, VALOR); // VALOR VALOR VALOR vet[4] vet[5] ...

// Gerar
struct c_unique { 
  int current;
  c_unique() {current=0;}
  int operator()() {return ++current;}
} UniqueNumber;

generate(vet.begin(), vet.end(), UniqueNumber);
generate_n(vet.begin(), 3, aleatorio);

// Remover elementos
it = remove(vet.begin(), vet.end(), 20); // "20 10 20 15" -> "10 15 ? ?". Retorna novo end
it = remove_if(ver.begin(), vet.end(), par);
remove_copy(vet.begin(), vet.end(), vet2.begin(), 20);
remove_copy_if(vet.begin(), vet.end(), vet2.begin(), par);

// Remover duplicatas consecutivas
it = unique(vet.begin(), vet.end(), <igual>); // "20 20 10 20" -> "20 10 20 ?". Retorna novo end
unique_copy(vet.begin(), vet.end(), vet2.begin(), <igual>); 

// Inverter
reverse(vet.begin(), vet.end());
reverse_copy(vet.begin(), vet.end(), vet2.end());

// Circular
rotate(vet.begin(), vet.begin() + 2, vet.end()); // "1 2 3 4" -> "3 4 1 2"
rotate_copy(vet.begin(), vet.begin() + 2, vet.end(), vet2.begin());

// Embaralhar
random_shuffle(vet.begin(), vet.end(), <aleatorio>);

// Separar elementos
it = partition(vet.begin(), vet.end(), par); // begin - it: pares; it - end: impares
it = stable_partition(vet.begin(), vet.end(), par); // mantem a ordem

// Ordenar
sort(vet.begin(), vet.end(), <menor>);
stable_sort(vet.begin(), vet.end(), <menor>); // mantem ordem de elementos semelhantes
partial_sort(vet.begin(), vet.begin() + 2, vet.end(), <menor>); // "4 3 2 1" -> "1 2 4 3"
partial_sort_copy(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <menor>);
nth_element(vet.begin(), vet.begin() + 5, vet.end(), <menor>); // vet[0..4] < vet[5] < vet[6..]

// Junção
merge(ordenado.begin(), ordenado.end(), ordenado2.begin(), ordenado2.end(), vet.begin());
inplace_merge(ordenado.begin(), ordenado.begin()+5, ordenado.end());
bool inclui = includes(ordenado.begin(), ordenado.end(), ordenado2.begin(), ordenado2.end());
it = set_union(ord.begin(), ord.end(), ord2.begin(), ord2.end(), vet.begin(), <menor>); // Merge sem repetição
it = set_intersection(ord.begin(), ord.end(), ord2.begin(), ord2.end(), vet.begin(), <menor>); // Interseção
it = set_difference(ord.begin(), ord.end(), ord2.begin(), ord2.end(), vet.begin(), <menor>); // Diferença
it = set_symmetric_difference(ord.begin(), ord.end(), ord2.begin(), ord2.end(), vet.begin(), <menor>); // (A - B) U (B - A)

// Heap
make_heap(vet.begin(), vet.end());
pop_head(vet.begin(), vet.end()); vet.pop_back();
vet.push_back(99); push_heap(vet.begin(), vet.end());
cout << "max: " << vet.front();
sort_heap(vet.begin(), vet.end());

// Min/max
v = min(VALOR, VALOR2);
v = max(VALOR, VALOR2);
it = min_element(vet.begin(), vet.end(), <menor>);
it = max_element(vet.begin(), vet.end(), <menor>);

// Comparação lexicográfica
int cmp = lexicographical_compare(vet.begin(), vet.end(), vet2.begin(), vet2.end(), <menor>); 

// Permutações, crescente
sort(vet.begin(), vet.end());
do {
	for_each(vet.begin(), vet.end(), imprime);
} while (next_permutation(vet.begin(), vet.end()));


// Permutações, decrescente
sort(vet.begin(), vet.end());
reverse(vet.begin(), vet.end());
do {
	for_each(vet.begin(), vet.end(), imprime);
} while (prev_permutation(vet.begin(), vet.end()));
```

## functional
```cpp
#include <functional>

// Aritmética / lógica
transform(vet.begin(), vet.end(), vet2.begin(), result.begin(), plus<TIPO>());
minus<TIPO>(); multiplies<TIPO>(); divides<TIPO>(); modulus<TIPO>(); negate<TIPO>();
logical_and<bool>(); logical_or<bool>(); logical_not<bool>();
// Comparação
it = mismatch(vet.begin(), vet.end(), vet2.begin(), equal_to<int>());
not_equal_to<TIPO>(); greater<TIPO>(); less<TIPO>(); greater_equal<TIPO>(); less_equal<TIPO>();
// Negação
count_if(vet.begin(), vet.end(), not1(par));
it = mismatch(vet.begin(), vet.end(), vet2.begin(), not2(equal_to<int>()));
// Bind
count_if(vet.begin(), vet.end(), bind1st(equal_to<int>(), 10));
count_if(vet.begin(), vet.end(), bind2nd(less<int>(), 10));
// Converte ponteiro em função
transform(vet.begin(), vet.end(), vet2.begin(), ptr_fun(atoi));
// Usar método de objeto
transform(vet.begin(), vet.end(), vet2.begin(), mem_fun(&string::length)); //se valores forem ponteiros
transform(vet.begin(), vet.end(), vet2.begin(), mem_fun_ref(&string::length)); //se valores forem valores

```

## utility
```cpp
#include <utility>
swap(a, b); 
pair<TIPO, TIPO> x = make_pair(a, b);
x.first; x.second;
```

## numeric
```cpp
#include <numeric>
// Acumular resultado - reduce
int v = accumulate(vet.begin(), vet.end(), INIT, <plus<int>()>); 
// Subtrair valores adjacentes
adjacent_difference(vet.begin(), vet.end(), vet2.begin(), <minus<int>()>);
// Acumular produtos entre adjacentes
int v = inner_product(vet.begin(), vet.end(), vet2.begin(), INIT, <plus<int>(), multiplies<int>()>)
// Soma parcial
partial_sum(vet.begin(), vet.end(), vet2.begin(), <plus<int>()>); // "1 2 3 4" -> "1 3 6 10"
```

## string
```cpp
#include <string>
string s;
string s("abc"); // construtor
string::iterator b = s.begin(), e = s.end(); // iteradores
int tamanho = s.size(); tamanho = s.length(); // tamanh
bool vazia = s.empty(); // vazia?
cout << c[1] << c.at(1); // acessar

s += "outra"; s.append("outra");
s.push_back('c');

char * cstr = s.c_str(); // converte para c string

// Busca
size_t posicao = s.find("valor", <POSICAO>); // primeira ocorrencia
size_t posicao = s.rfind("valor", <POSICAO>); // ultima ocorrencia 
size_t posicao = s.find_first_of("aeiou"); // algum dos chars
size_t posicao = s.find_last_of("aeiou"); // algum dos chars
size_t posicao = s.find_first_not_of("aeiou"); // algum que não é um dos chars
size_t posicao = s.find_last_not_of("aeiou"); // algum que não é um dos chars

bool achou = posicao != string::npos;

// Substring
string s2 = s.substr(INICIO, <TAMANHO>); 

// Comparar
int cmp = compare(<INICIO, TAMANHO>, outra, <INICIO, TAMANHO>);

// Substituir
s.replace(INICIO, TAMANHO, outra); 
s.replace(INICIO, TAMANHO, outra, INICIO, TAMANHO);
s.replace(INICIO, TAMANHO, outra, TAMANHO);
s.replace(INICIO, TAMANHO, QUANTIDADE, CHAR);

// Ler linha
getline(cin, s);
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

v = ceil(12.3);  // 13
v = floor(12.3); // 12
v = abs(-3.4);   // 3.4, Módulo

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

## limits
```cpp
#include <limits>
// Exemplos para int
numeric_limits<TIPO>::min(); // -2147483648
numeric_limits<TIPO>::max(); // 21477483647
numeric_limits<TIPO>::is_signed(); // 1
numeric_limits<TIPO>::digits(); // 31 bits
numeric_limits<TIPO>::has_infinity; // 0
```

## complex
```cpp
#include <complex>

complex<double> x(REAL, IMAG);
real(x); imag(x); // Pega parte real e imaginária
abs(x); arg(x) // x = abs(x)*e^i*arg(x) rad
norm(x); // Norma de x
conjugate(x); // Conjugado

x = polar(2, 0.5); x = 2*e^i*0.5
```

## BFS
```cpp

void bfs(T src, T dest){
	queue<T> fila;
	fila.push_back(src);
	set<T> visitados;

	while(!fila.empty()){
		T cur = fila.front();

		if(cur == dest){
			// retorna
		}

		fila.pop();
		visitados.insert(cur);
		foreach(T adj in cur.adjacencias){
			if(!visitados.contains(adj)){ // Pseudo-code
				fila.push_back(adj);
				visitados.insert(adj);
			}
		}
	}
}
```


# TODO ALG
- djasktra
- bellman-ford
- floyd-marshall
- kruskal
- prim
- lcs
- lis
- modular_pow
