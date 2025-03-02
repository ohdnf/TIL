import React from "react";

const USERS_URL = "https://example.com/api/users";

class Table extends React.Component {
  state = {
    users: [],
    count: 0,
    pageSize: 10,
    currentPage: 0,
    maxPage: 0,
  };

  fetchUsers(page) {
    fetch(USERS_URL + `?page=${page}`)
      .then((res) => res.json())
      .then((res) => {
        console.log(res.count);
        const maxPage = parseInt((count - 1) / 10);
        console.log(maxPage);
        this.setState({
          users: res.results,
          count: res.count,
          maxPage: maxPage,
        });
      })
      .catch((err) => console.error(err));
  }

  goFirstPage(page) {
    this.setState({
      currentPage: 0,
    });
    fetchUsers(0);
  }

  goPrevPage(page) {
    if (page > 0) {
      this.setState({
        currentPage: currentPage - 1,
      });
    }
    fetchUsers(page - 1);
  }

  goNextPage(page) {
    if (page < maxPage) {
      this.setState({
        currentPage: currentPage + 1,
      });
    }
    fetchUsers(page + 1);
  }

  goLastPage(page) {
    if (page === maxPage) {
      return;
    }
    this.setState({
      currentPage: currentPage + 1,
    });
    fetchUsers(maxPage);
  }

  componentDidMount() {
    this.fetchUsers(this.state.currentPage);
  }

  render() {
    const { users, count, currentPage, maxPage } = this.state;

    const listUsers = users.map((user) => {
      <tr key={user.id}>
        <td>{user.id}</td>
        <td>{user.firstName}</td>
        <td>{user.lastName}</td>
      </tr>;
    });
    return (
      <div>
        <table className="table">
          <thead>
            <tr>
              <th>ID</th>
              <th>First Name</th>
              <th>Last Name</th>
            </tr>
          </thead>
          <tbody>
            // render elements in tbody
            {listUsers}
          </tbody>
        </table>
        <section className="pagination">
          <button
            className="first-page-btn"
            onClick={() => goFirstPage(currentPage)}
            disable={currentPage === 0}
          >
            first
          </button>
          <button
            className="previous-page-btn"
            onClick={() => goPrevPage(currentPage)}
            disable={currentPage > maxPage}
          >
            previous
          </button>
          <button
            className="next-page-btn"
            onClick={() => goNextPage(currentPage)}
            disable={currentPage < maxPage}
          >
            next
          </button>
          <button
            className="last-page-btn"
            onClick={() => goLastPage(currentPage)}
            disable={currentPage >= maxPage}
          >
            last
          </button>
        </section>
      </div>
    );
  }
}

export default Table;
