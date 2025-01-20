package main

import (
	"bufio"
	"container/heap"
	"fmt"
	"os"
	"strconv"
	"strings"
)

// ------------------------------
// Структуры и типы
// ------------------------------

// Cell описывает координаты клетки (row, col).
type Cell struct {
	row, col int
}

// Node — элемент для приоритетной очереди (хранит стоимость и координаты).
type Node struct {
	cost int
	cell Cell
}

// PriorityQueue реализует кучу (min-heap) для Node.
type PriorityQueue []*Node

func (pq *PriorityQueue) Len() int           { return len(*pq) }
func (pq *PriorityQueue) Less(i, j int) bool { return (*pq)[i].cost < (*pq)[j].cost }
func (pq *PriorityQueue) Swap(i, j int)      { (*pq)[i], (*pq)[j] = (*pq)[j], (*pq)[i] }
func (pq *PriorityQueue) Push(x interface{}) { *pq = append(*pq, x.(*Node)) }
func (pq *PriorityQueue) Pop() interface{} {
	old := *pq
	n := len(old)
	item := old[n-1]
	*pq = old[0 : n-1]
	return item
}

// fatalError печатает сообщение об ошибке в STDERR и завершает программу с кодом 1.
func fatalError(msg string) {
	// Печатаем в стандартный поток ошибок:
	fmt.Fprintln(os.Stderr, msg)
	os.Exit(1)
}

// parseInt конвертирует строку в int, при ошибке — вызывает fatalError.
func parseInt(s string) int {
	val, err := strconv.Atoi(s)
	if err != nil {
		fatalError(fmt.Sprintf("Ошибка при преобразовании числа '%s': %v", s, err))
	}
	return val
}

// buildPath восстанавливает путь из массива prevCell, двигаясь от финиша к старту.
func buildPath(prevCell [][]*Cell, start, finish Cell) []Cell {
	path := []Cell{}
	cur := &finish
	for cur != nil {
		path = append(path, *cur)
		if *cur == start {
			break
		}
		cur = prevCell[cur.row][cur.col]
	}

	// Путь восстанавливается "с конца" к "началу", поэтому инвертируем
	for i, j := 0, len(path)-1; i < j; i, j = i+1, j-1 {
		path[i], path[j] = path[j], path[i]
	}
	return path
}

// findShortestPathDijkstra ищет кратчайший путь (по суммарной стоимости) в матрице labyrinth
// из клетки start в клетку finish. Если путь не найден, возвращает nil.
func findShortestPathDijkstra(labyrinth [][]int, start, finish Cell) []Cell {
	rows := len(labyrinth)
	cols := len(labyrinth[0])

	// Если старт или финиш — это стена (0), путь невозможен.
	if labyrinth[start.row][start.col] == 0 {
		return nil
	}
	if labyrinth[finish.row][finish.col] == 0 {
		return nil
	}

	// Массив расстояний (стоимостей).
	distances := make([][]int, rows)
	for i := 0; i < rows; i++ {
		distances[i] = make([]int, cols)
		for j := 0; j < cols; j++ {
			distances[i][j] = int(^uint(0) >> 1) // Int max
		}
	}
	distances[start.row][start.col] = labyrinth[start.row][start.col]

	// Массив для восстановления пути:
	prevCell := make([][]*Cell, rows)
	for i := 0; i < rows; i++ {
		prevCell[i] = make([]*Cell, cols)
	}

	// Приоритетная очередь (min-heap).
	pq := &PriorityQueue{}
	heap.Init(pq)
	heap.Push(pq, &Node{
		cost: distances[start.row][start.col],
		cell: start,
	})

	// Векторы направления (вверх, вниз, влево, вправо).
	dirs := []struct{ dr, dc int }{
		{-1, 0}, {1, 0}, {0, -1}, {0, 1},
	}

	// Алгоритм Дейкстры
	for pq.Len() > 0 {
		node := heap.Pop(pq).(*Node)
		currentCell := node.cell
		currentCost := node.cost

		// Если уже пришли к финишу — восстанавливаем путь и выходим.
		if currentCell == finish {
			return buildPath(prevCell, start, finish)
		}

		// Если стоимость больше актуальной, пропускаем.
		if currentCost > distances[currentCell.row][currentCell.col] {
			continue
		}

		// Перебираем соседей.
		for _, d := range dirs {
			nr := currentCell.row + d.dr
			nc := currentCell.col + d.dc

			// Проверка границ.
			if nr < 0 || nr >= rows || nc < 0 || nc >= cols {
				continue
			}
			// Стены (0) не проходим.
			if labyrinth[nr][nc] == 0 {
				continue
			}

			// Новая потенциальная стоимость (sum of costs).
			newCost := currentCost + labyrinth[nr][nc]
			if newCost < distances[nr][nc] {
				distances[nr][nc] = newCost
				prevCell[nr][nc] = &Cell{currentCell.row, currentCell.col}
				heap.Push(pq, &Node{cost: newCost, cell: Cell{nr, nc}})
			}
		}
	}

	// Путь не найден.
	return nil
}

func main() {
	scanner := bufio.NewScanner(os.Stdin)

	// Считываем размеры лабиринта.
	if !scanner.Scan() {
		fatalError("Не удалось прочитать строку с размерами лабиринта")
	}
	sizeLine := scanner.Text()
	sizeParts := strings.Split(sizeLine, " ")
	if len(sizeParts) != 2 {
		fatalError("Некорректный формат размеров лабиринта (ожидалось 2 числа)")
	}
	rows := parseInt(sizeParts[0])
	cols := parseInt(sizeParts[1])
	if rows <= 0 || cols <= 0 {
		fatalError("Размеры лабиринта должны быть положительными")
	}

	// Считываем сам лабиринт (matrix).
	labyrinth := make([][]int, rows)
	for i := 0; i < rows; i++ {
		if !scanner.Scan() {
			fatalError(fmt.Sprintf("Не удалось прочитать строку №%d лабиринта", i))
		}
		rowLine := scanner.Text()
		rowParts := strings.Split(rowLine, " ")
		if len(rowParts) != cols {
			fatalError(fmt.Sprintf("Число столбцов в строке №%d не совпадает с %d", i, cols))
		}
		labyrinth[i] = make([]int, cols)
		for j := 0; j < cols; j++ {
			labyrinth[i][j] = parseInt(rowParts[j])
			if labyrinth[i][j] < 0 || labyrinth[i][j] > 9 {
				fatalError(fmt.Sprintf("Неверное число в лабиринте (должно быть 0..9), строка %d, столбец %d", i, j))
			}
		}
	}

	// Считываем координаты старта и финиша.
	if !scanner.Scan() {
		fatalError("Не удалось прочитать строку с координатами старта и финиша")
	}
	coordsLine := scanner.Text()
	coordsParts := strings.Split(coordsLine, " ")
	if len(coordsParts) != 4 {
		fatalError("Неверный формат координат (ожидалось 4 числа)")
	}

	startRow := parseInt(coordsParts[0])
	startCol := parseInt(coordsParts[1])
	finishRow := parseInt(coordsParts[2])
	finishCol := parseInt(coordsParts[3])

	// Проверка границ.
	if startRow < 0 || startRow >= rows || startCol < 0 || startCol >= cols {
		fatalError(fmt.Sprintf("Координаты старта (%d, %d) вне границ лабиринта", startRow, startCol))
	}
	if finishRow < 0 || finishRow >= rows || finishCol < 0 || finishCol >= cols {
		fatalError(fmt.Sprintf("Координаты финиша (%d, %d) вне границ лабиринта", finishRow, finishCol))
	}

	startCell := Cell{startRow, startCol}
	finishCell := Cell{finishRow, finishCol}

	// Поиск кратчайшего пути (Дейкстра).
	path := findShortestPathDijkstra(labyrinth, startCell, finishCell)
	if path == nil {
		fatalError("Нет пути от старта до финиша")
	}

	// Вывод результата в STDOUT.
	for _, c := range path {
		fmt.Printf("%d %d\n", c.row, c.col)
	}
	fmt.Println(".")
}
